#!/usr/bin/env python3
"""Local camera + Intel RealSense D415 dashboard.

Owns USB RGB cameras via OpenCV and D415 color/depth via pyrealsense2.
Skips the MacBook FaceTime camera. Does not proxy any other local services.

On macOS the two camera families need different privileges, so run two instances:
       ./supervise.sh --detach uvc       8090   # C922 + wrist cam (Camera permission is per user)
  sudo ./supervise.sh --detach realsense 8091   # D415 depth + RGB (librealsense needs root)
DASH_PORT / DASH_MODE env vars select port and camera family (see MODE below).
"""

from __future__ import annotations

import ctypes
import json
import os
import re
import subprocess
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import cv2
import numpy as np
import uvicorn
from cv2_enumerate_cameras import enumerate_cameras
from fastapi import FastAPI
from fastapi.responses import FileResponse, JSONResponse, Response, StreamingResponse
from fastapi.staticfiles import StaticFiles

try:
    import pyrealsense2 as rs
except ImportError:  # pragma: no cover
    rs = None

ROOT = Path(__file__).resolve().parent
STATIC = ROOT / "static"
HOST = "127.0.0.1"
PORT = int(os.environ.get("DASH_PORT", "8090"))
IS_ROOT = hasattr(os, "geteuid") and os.geteuid() == 0


def _resolve_mode() -> str:
    """Which cameras this process owns.

    all       - UVC/AVFoundation cameras + D415 via pyrealsense2 in one process.
    realsense - D415 depth + colour only. On macOS librealsense can only claim the D415's
                USB interfaces as root, so this instance runs under sudo.
    uvc       - OpenCV/AVFoundation cameras only; never touches pyrealsense2. Must run as the
                logged-in user: macOS Camera permission is per user and root has none.
    auto      - root -> realsense; non-root on macOS -> uvc; elsewhere -> all.
    """
    mode = os.environ.get("DASH_MODE", "auto").strip().lower()
    if mode == "auto":
        if IS_ROOT:
            return "realsense"
        return "uvc" if os.uname().sysname == "Darwin" else "all"
    if mode not in {"all", "realsense", "uvc"}:
        raise SystemExit(f"DASH_MODE must be auto|all|realsense|uvc, got {mode!r}")
    return mode


MODE = _resolve_mode()
WANT_UVC = MODE in {"all", "uvc"}
WANT_D415 = MODE in {"all", "realsense"}
JPEG_QUALITY = 80
STALE_AFTER_S = 2.5
RGB_RETRY_S = 3.0
DEPTH_RETRY_S = 30.0
D415_START_WAIT_MS = 120_000
D415_FRAME_WAIT_MS = 30_000
D415_STREAM_MISS_LIMIT = 40
ARM_SERIAL = "/dev/cu.usbmodem5A7A0545771"

KNOWN_DEVICES = (
    {
        "match": "c922",
        "label": "Logitech C922 · canvas",
        "role": "Far side of drawing paper",
        "size": (1280, 720),
    },
    {
        "match": "usb2.0_cam1",
        "label": "SO-101 wrist camera",
        "role": "Pen / gripper close-up",
        # 720p starves this camera: it shares the USB 2 bus with the C922 and stops
        # delivering frames after a few seconds.
        "size": (640, 480),
    },
    {
        "match": "realsense",
        "label": "RealSense D415 RGB",
        "role": "Depth camera color",
    },
)

# Colour + depth first so the dashboard gets both D415 tiles; depth-only profiles are the
# fallback (e.g. when the camera is on a USB 2 link and the combined profile fails).
STREAM_ATTEMPTS = (
    {"color": (640, 480, 15), "depth": (640, 480, 15)},
    {"color": (424, 240, 15), "depth": (480, 270, 15)},
    {"depth": (640, 480, 15)},
    {"depth": (480, 270, 15)},
)


def _now() -> float:
    return time.time()


def _slug(name: str, unique: str) -> str:
    base = re.sub(r"[^a-z0-9]+", "-", name.lower()).strip("-") or "camera"
    tail = re.sub(r"[^a-z0-9]+", "", unique.lower())[-8:] or "id"
    return f"{base}-{tail}"


def _annotate(name: str) -> dict[str, Any]:
    lowered = name.lower()
    for known in KNOWN_DEVICES:
        if known["match"] in lowered:
            return dict(known)
    return {"match": None, "label": name, "role": "Discovered camera", "size": (640, 480)}


def _wrap_text(message: str, width: int = 40) -> list[str]:
    words = message.split()
    lines: list[str] = []
    current = ""
    for word in words:
        trial = f"{current} {word}".strip()
        if len(trial) <= width:
            current = trial
        else:
            if current:
                lines.append(current)
            current = word
    if current:
        lines.append(current)
    return lines or ["No signal"]


def _placeholder(message: str, subtitle: str = "") -> bytes:
    img = np.zeros((360, 640, 3), dtype=np.uint8)
    img[:] = (14, 16, 22)
    cv2.rectangle(img, (16, 16), (623, 343), (42, 52, 68), 1)
    y = 150
    for line in _wrap_text(message)[:5]:
        cv2.putText(img, line, (36, y), cv2.FONT_HERSHEY_SIMPLEX, 0.58, (180, 196, 220), 1, cv2.LINE_AA)
        y += 30
    if subtitle:
        cv2.putText(img, subtitle[:58], (36, 300), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (90, 140, 160), 1, cv2.LINE_AA)
    ok, buf = cv2.imencode(".jpg", img, [int(cv2.IMWRITE_JPEG_QUALITY), 70])
    return buf.tobytes() if ok else b""


def _encode_bgr(frame: np.ndarray) -> bytes | None:
    ok, buf = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), JPEG_QUALITY])
    return buf.tobytes() if ok else None


def _skip_uvc_camera(name: str) -> bool:
    lowered = name.lower()
    # Skip FaceTime. Skip D415 as a generic UVC RGB camera because the SDK
    # thread owns color+depth; opening it here hides the device from librealsense.
    return (
        "facetime" in lowered
        or "realsense" in lowered
        or "depth camera 415" in lowered
    )


def _keep_libusb_alive():
    if not rs:
        return None
    dylib = Path(rs.__file__).parent / ".dylibs" / "libusb-1.0.0.dylib"
    if not dylib.exists():
        return None
    usb = ctypes.CDLL(str(dylib))
    usb.libusb_init.argtypes = [ctypes.POINTER(ctypes.c_void_p)]
    usb.libusb_init.restype = ctypes.c_int
    ctx = ctypes.c_void_p()
    code = usb.libusb_init(ctypes.byref(ctx))
    return {"usb": usb, "context": ctx, "status": code, "ok": code == 0 and bool(ctx.value)}


def _run(cmd: list[str], timeout: float = 8.0) -> str:
    try:
        proc = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
        return proc.stdout or proc.stderr or ""
    except (OSError, subprocess.SubprocessError):
        return ""


def discover_usb_cameras() -> tuple[list[dict[str, Any]], list[dict[str, Any]]]:
    cameras: list[dict[str, Any]] = []
    raw = _run(["/usr/sbin/system_profiler", "SPCameraDataType", "-json"], timeout=12)
    try:
        payload = json.loads(raw) if raw.strip().startswith("{") else {}
    except json.JSONDecodeError:
        payload = {}
    for item in payload.get("SPCameraDataType") or []:
        name = item.get("_name") or ""
        if "facetime" in name.lower():
            continue
        cameras.append(
            {
                "name": name,
                "model_id": item.get("spcamera_model-id"),
                "unique_id": item.get("spcamera_unique-id"),
            }
        )
    ioreg = _run(["/usr/sbin/ioreg", "-p", "IOUSB", "-l", "-w", "0"], timeout=8)
    usb_rows: list[dict[str, Any]] = []
    for block in re.split(r"\+-o ", ioreg):
        product = re.search(r'"kUSBProductString" = "([^"]+)"', block) or re.search(
            r'"USB Product Name" = "([^"]+)"', block
        )
        if not product:
            continue
        vid = re.search(r'"idVendor" = (\d+)', block)
        pid = re.search(r'"idProduct" = (\d+)', block)
        speed = re.search(r'"Device Speed" = (\d+)', block)
        loc = re.search(r'"locationID" = (\d+)', block)
        serial = re.search(r'"kUSBSerialNumberString" = "([^"]+)"', block)
        usb_rows.append(
            {
                "product": product.group(1),
                "vid": int(vid.group(1)) if vid else None,
                "pid": int(pid.group(1)) if pid else None,
                "device_speed": int(speed.group(1)) if speed else None,
                "location_id": int(loc.group(1)) if loc else None,
                "serial": serial.group(1) if serial else None,
            }
        )
    return cameras, usb_rows


def match_usb(name: str, vid: int | None, pid: int | None, usb_rows: list[dict[str, Any]]) -> dict[str, Any] | None:
    lowered = name.lower()
    for row in usb_rows:
        product = (row.get("product") or "").lower()
        if vid and pid and row.get("vid") == vid and row.get("pid") == pid:
            return row
        if product and (product in lowered or lowered.split()[0] in product):
            return row
    return None


def usb_speed_label(speed: int | None) -> str | None:
    return {1: "USB 1.1", 2: "USB 2.0", 3: "USB 3.0+"}.get(speed) if speed else None


@dataclass
class StreamState:
    id: str
    name: str
    kind: str
    backend: str
    device_id: str | None = None
    index: int | None = None
    vid: int | None = None
    pid: int | None = None
    unique_id: str | None = None
    role: str = ""
    width: int = 0
    height: int = 0
    requested_fps: float = 0.0
    fps: float = 0.0
    frames: int = 0
    last_frame_ts: float | None = None
    error: str | None = "Waiting for first frame"
    jpeg: bytes | None = None
    extra: dict[str, Any] = field(default_factory=dict)
    lock: threading.Lock = field(default_factory=threading.Lock)

    def snapshot(self) -> dict[str, Any]:
        with self.lock:
            last = self.last_frame_ts
            age = (_now() - last) if last else None
            if self.error and not last:
                health = "error"
            elif last and age is not None and age <= STALE_AFTER_S:
                health = "live"
            elif last:
                health = "stale"
            else:
                health = "waiting"
            usb = None
            if self.vid is not None and self.pid is not None:
                usb = f"{self.vid:04X}:{self.pid:04X}"
            return {
                "id": self.id,
                "name": self.name,
                "role": self.role,
                "kind": self.kind,
                "backend": self.backend,
                "device_id": self.device_id,
                "index": self.index,
                "unique_id": self.unique_id,
                "usb_ids": usb,
                "vid": self.vid,
                "pid": self.pid,
                "width": self.width,
                "height": self.height,
                "fps": round(self.fps, 2),
                "requested_fps": self.requested_fps,
                "frames": self.frames,
                "last_frame_unix": last,
                "last_frame_age_s": round(age, 3) if age is not None else None,
                "health": health,
                "error": self.error,
                "has_frame": self.jpeg is not None,
                **self.extra,
            }

    def publish(self, jpeg: bytes, width: int, height: int, extra: dict[str, Any] | None = None) -> None:
        now = _now()
        with self.lock:
            if self.last_frame_ts:
                dt = now - self.last_frame_ts
                if dt > 0:
                    inst = 1.0 / dt
                    self.fps = inst if self.frames < 4 else (0.85 * self.fps + 0.15 * inst)
            self.jpeg = jpeg
            self.width = width
            self.height = height
            self.frames += 1
            self.last_frame_ts = now
            self.error = None
            if extra:
                self.extra.update(extra)

    def set_error(self, message: str) -> None:
        with self.lock:
            self.error = message

    def current_jpeg(self) -> bytes:
        with self.lock:
            if self.jpeg:
                return self.jpeg
            return _placeholder(self.error or "No signal", self.name)


class CameraHub:
    def __init__(self) -> None:
        self.streams: dict[str, StreamState] = {}
        self.running = False
        self._threads: list[threading.Thread] = []
        self._system: dict[str, Any] = {}
        self._system_lock = threading.Lock()
        self.started_at = _now()
        self._libusb = None
        self._d415_last_reset = 0.0

    def start(self) -> None:
        self.running = True
        if WANT_D415:
            self._libusb = _keep_libusb_alive()
        self.refresh_system()
        if WANT_UVC:
            self._start_rgb()
            threading.Thread(target=self._rgb_rescan_loop, name="rgb-rescan", daemon=True).start()
        proxy = os.environ.get("D415_PROXY", "").rstrip("/")
        if proxy:
            # another dashboard (e.g. one running as root, which is what librealsense needs on macOS)
            # owns the D415; mirror its depth + colour snapshots here so one page shows every camera
            for sid in ("realsense-depth", "realsense-rgb"):
                state = self._ensure_depth_stream() if sid == "realsense-depth" else self._ensure_d415_rgb()
                state.backend = f"proxied from {proxy}"
                threading.Thread(target=self._proxy_loop, args=(state, f"{proxy}/snapshot/{sid}"),
                                 name=f"proxy-{sid}", daemon=True).start()
        elif WANT_D415:
            self._ensure_d415_rgb()
            self._ensure_depth_stream()
            threading.Thread(target=self._d415_loop, name="realsense-d415", daemon=True).start()
        # Do not open D415 as UVC. That exclusive-opens the composite device
        # and librealsense then reports 0 SDK devices.
        threading.Thread(target=self._system_loop, name="system-refresh", daemon=True).start()

    def stop(self) -> None:
        self.running = False

    def refresh_system(self) -> None:
        profiler, usb_rows = discover_usb_cameras()
        info = {
            "host": HOST,
            "port": PORT,
            "url": f"http://{HOST}:{PORT}",
            "started_unix": self.started_at,
            "uptime_s": round(_now() - self.started_at, 1),
            "so101_serial": ARM_SERIAL,
            "so101_present": os.path.exists(ARM_SERIAL),
            "pyrealsense2": bool(rs),
            "mode": MODE,
            "uid": os.getuid(),
            "profiler_cameras": profiler,
            "usb_devices": usb_rows,
            "realsense_sdk_devices": self._sdk_device_count(),
            "d415_source": f"local pyrealsense2 on :{PORT}" if WANT_D415 else "not owned by this instance (mode=uvc)",
            "libusb_ok": bool(self._libusb and self._libusb.get("ok")),
        }
        with self._system_lock:
            self._system = info
        return info

    def system_snapshot(self) -> dict[str, Any]:
        with self._system_lock:
            data = dict(self._system)
        data["uptime_s"] = round(_now() - self.started_at, 1)
        data["stream_count"] = len(self.streams)
        data["live_count"] = sum(1 for s in self.list_streams() if s["health"] == "live")
        return data

    def list_streams(self) -> list[dict[str, Any]]:
        return [s.snapshot() for s in self.streams.values()]

    def get(self, stream_id: str) -> StreamState | None:
        return self.streams.get(stream_id)

    def _sdk_device_count(self) -> int | None:
        if not rs or not WANT_D415:
            # In uvc mode never touch librealsense: enumerating would contend with the
            # root realsense instance for the device.
            return None
        try:
            return int(len(rs.context().query_devices()))
        except Exception:
            return None

    def _start_rgb(self) -> None:
        _, usb_rows = discover_usb_cameras()
        found = list(enumerate_cameras())
        started = 0
        for cam in found:
            name = cam.name or f"Camera {cam.index}"
            if _skip_uvc_camera(name):
                continue
            meta = _annotate(name)
            unique = getattr(cam, "path", None) or str(cam.index)
            stream_id = _slug(name, unique)
            if stream_id in self.streams:
                continue
            usb = match_usb(name, cam.vid, cam.pid, usb_rows)
            state = StreamState(
                id=stream_id,
                name=meta["label"],
                kind="rgb",
                backend="OpenCV AVFoundation",
                device_id=unique,
                index=int(cam.index),
                vid=cam.vid,
                pid=cam.pid,
                unique_id=unique,
                role=meta["role"],
                extra={
                    "capture_size": list(meta.get("size") or (640, 480)),
                    "raw_name": name,
                    "usb_speed": usb_speed_label(usb.get("device_speed") if usb else None),
                    "usb_product": usb.get("product") if usb else None,
                    "usb_location_id": usb.get("location_id") if usb else None,
                },
            )
            self.streams[stream_id] = state
            thread = threading.Thread(target=self._rgb_loop, args=(state,), name=f"rgb-{stream_id}", daemon=True)
            thread.start()
            self._threads.append(thread)
            started += 1
            self.streams.pop("no-uvc", None)
        if started == 0 and "no-uvc" not in self.streams and not any(
            sid not in {"realsense-rgb", "realsense-depth"} and st.kind == "rgb"
            for sid, st in self.streams.items()
        ):
            placeholder = StreamState(
                id="no-uvc",
                name="No USB RGB cameras",
                kind="rgb",
                backend="none",
                error="C922 / USB2.0_CAM1 were not enumerated",
            )
            placeholder.jpeg = _placeholder("No USB RGB cameras")
            self.streams[placeholder.id] = placeholder

    def _rgb_rescan_loop(self) -> None:
        while self.running:
            time.sleep(4)
            try:
                self._start_rgb()
            except Exception:
                pass

    def _ensure_d415_rgb(self) -> StreamState:
        existing = self.streams.get("realsense-rgb")
        if existing:
            return existing
        state = StreamState(
            id="realsense-rgb",
            name="RealSense D415 RGB",
            kind="rgb",
            backend="pyrealsense2 color",
            vid=0x8086,
            pid=0x0AD3,
            role="Depth camera color",
            extra={"sensor": "Intel RealSense D415", "allow_browser_capture": False},
        )
        self.streams[state.id] = state
        return state

    def _ensure_depth_stream(self) -> StreamState:
        existing = self.streams.get("realsense-depth")
        if existing:
            return existing
        state = StreamState(
            id="realsense-depth",
            name="RealSense D415 depth",
            kind="depth",
            backend="pyrealsense2 depth" if rs else "unavailable",
            role="Overhead / scene depth",
            extra={"units": "mm", "sensor": "Intel RealSense D415", "allow_browser_capture": False},
        )
        self.streams[state.id] = state
        return state

    def _rgb_loop(self, state: StreamState) -> None:
        while self.running:
            cap = cv2.VideoCapture(state.index)
            if not cap.isOpened():
                state.set_error(f"Could not open OpenCV index {state.index}")
                time.sleep(RGB_RETRY_S)
                continue
            cap_w, cap_h = state.extra.get("capture_size") or (640, 480)
            # MJPG + a 30 fps cap: both webcams and the D415 share one USB 2 bus, and an
            # uncompressed 720p60 stream alone exceeds its bandwidth (the wrist camera then
            # stops delivering frames a few seconds after it opens).
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, int(cap_w))
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, int(cap_h))
            cap.set(cv2.CAP_PROP_FPS, 30)
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            backend = cap.getBackendName() or state.backend
            requested = float(cap.get(cv2.CAP_PROP_FPS) or 0)
            with state.lock:
                state.backend = f"OpenCV {backend}"
                state.requested_fps = requested
            failures = 0
            while self.running:
                ok, frame = cap.read()
                if not ok or frame is None:
                    failures += 1
                    if failures >= 12:
                        state.set_error("Camera opened but is not delivering frames")
                        break
                    time.sleep(0.08)
                    continue
                failures = 0
                jpeg = _encode_bgr(frame)
                if jpeg:
                    h, w = frame.shape[:2]
                    state.publish(jpeg, w, h)
            cap.release()
            if self.running:
                time.sleep(RGB_RETRY_S)

    def _proxy_loop(self, state: StreamState, url: str) -> None:
        """Mirror another dashboard's snapshot endpoint (~5 fps) into this stream."""
        import json as _json
        import urllib.request

        base = url.split("/snapshot/")[0]
        n = 0
        while self.running:
            try:
                n += 1
                if n % 10 == 1:  # every ~2 s: mirror the upstream health so an upstream error shows as ours
                    with urllib.request.urlopen(f"{base}/api/streams", timeout=3) as resp:
                        rows = _json.loads(resp.read()).get("streams", [])
                    up = next((r for r in rows if r.get("id") == state.id), None)
                    if up is None or up.get("health") != "live":
                        raise RuntimeError(f"upstream {up.get('health') if up else 'missing'}: {(up or {}).get('error') or ''}")
                req = urllib.request.Request(f"{url}?t={int(_now() * 1000)}")
                with urllib.request.urlopen(req, timeout=3) as resp:
                    jpeg = resp.read()
                if resp.status != 200 or not jpeg.startswith(b"\xff\xd8"):
                    raise RuntimeError(f"upstream status {resp.status}")
                arr = np.frombuffer(jpeg, dtype=np.uint8)
                frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)
                if frame is None:
                    raise RuntimeError("upstream frame did not decode")
                h, w = frame.shape[:2]
                state.publish(jpeg, w, h, {"d415_source": f"proxied from {url}"})
                time.sleep(0.2)
            except Exception as exc:
                state.set_error(f"proxy: {exc}")
                time.sleep(2.0)

    def _d415_loop(self) -> None:
        rgb = self._ensure_d415_rgb()
        depth = self._ensure_depth_stream()
        while self.running:
            try:
                self._run_sdk_color_depth(rgb, depth)
            except Exception as exc:
                print(f"D415 session error: {exc}", flush=True)
                depth.set_error(str(exc))
                if rgb.snapshot().get("health") != "live":
                    rgb.set_error(str(exc))
            if self.running:
                time.sleep(DEPTH_RETRY_S)

    def _run_sdk_color_depth(self, rgb: StreamState, depth: StreamState) -> None:
        if not rs:
            depth.set_error("pyrealsense2 is not installed")
            return
        ctx = rs.context()
        devices = ctx.query_devices()
        if len(devices) == 0:
            usb_note = self._realsense_usb_note()
            depth.set_error(
                "librealsense sees 0 D415 devices. Another process may own the SDK. " + usb_note
            )
            time.sleep(DEPTH_RETRY_S)
            return

        device = devices[0]
        name = device.get_info(rs.camera_info.name)
        serial = device.get_info(rs.camera_info.serial_number)
        usb_type = (
            device.get_info(rs.camera_info.usb_type_descriptor)
            if device.supports(rs.camera_info.usb_type_descriptor)
            else None
        )
        print(f"D415 starting pipeline {serial}", flush=True)
        pipeline, profile, last_error = self._start_d415_pipeline(ctx, serial)
        if profile is None:
            raise RuntimeError(last_error)

        try:
            depth_sensor = profile.get_device().first_depth_sensor()
            scale = float(depth_sensor.get_depth_scale())
            colorizer = rs.colorizer()
            identity = {
                "sdk_name": name,
                "serial": serial,
                "usb_type": usb_type,
                "depth_scale_m": scale,
                "d415_source": f"local pyrealsense2 on :{PORT}",
                "allow_browser_capture": False,
            }
            with depth.lock:
                depth.backend = "pyrealsense2 depth"
                depth.device_id = serial
                depth.vid = 0x8086
                depth.pid = 0x0AD3
                depth.extra.update(identity)
            first = True
            misses = 0
            while self.running:
                wait_ms = D415_START_WAIT_MS if first else D415_FRAME_WAIT_MS
                ok, frames = pipeline.try_wait_for_frames(wait_ms)
                if not ok:
                    misses += 1
                    depth.set_error(f"Waiting for D415 frames ({misses}/{D415_STREAM_MISS_LIMIT})")
                    if not first and misses >= D415_STREAM_MISS_LIMIT:
                        raise RuntimeError("D415 pipeline timed out waiting for frames")
                    continue
                first = False
                misses = 0
                color = frames.get_color_frame()
                depth_frame = frames.get_depth_frame()
                if color:
                    color_bgr = np.asanyarray(color.get_data())
                    jpeg = _encode_bgr(color_bgr)
                    if jpeg:
                        h, w = color_bgr.shape[:2]
                        extra = dict(identity)
                        with rgb.lock:
                            rgb.backend = "pyrealsense2 color"
                            rgb.device_id = serial
                            rgb.vid = 0x8086
                            rgb.pid = 0x0AD3
                        rgb.publish(jpeg, w, h, extra)
                if depth_frame:
                    raw = np.asanyarray(depth_frame.get_data())
                    extra = self._depth_stats(raw, scale, depth_frame)
                    extra.update(identity)
                    try:
                        colorized = np.asanyarray(colorizer.colorize(depth_frame).get_data())
                        if colorized.ndim == 3 and colorized.shape[2] == 3:
                            bgr = cv2.cvtColor(colorized, cv2.COLOR_RGB2BGR)
                        else:
                            bgr = self._colorize_numpy(raw)
                    except Exception:
                        bgr = self._colorize_numpy(raw)
                    jpeg = _encode_bgr(bgr)
                    if jpeg:
                        h, w = bgr.shape[:2]
                        depth.publish(jpeg, w, h, extra)
        finally:
            try:
                pipeline.stop()
            except Exception:
                pass

    def _d415_rgb_uvc_loop(self) -> None:
        """RGB via Camera permission only if the SDK did not already start color."""
        rgb = self._ensure_d415_rgb()
        # Let the SDK claim the composite device first. Opening D415 as UVC
        # hides it from librealsense on this Mac.
        delay_until = time.monotonic() + 15
        while self.running and time.monotonic() < delay_until:
            time.sleep(0.4)
        while self.running:
            snap = rgb.snapshot()
            if snap.get("health") == "live" and "pyrealsense2" in (snap.get("backend") or ""):
                time.sleep(2)
                continue
            index = None
            for cam in enumerate_cameras():
                name = (cam.name or "").lower()
                if "realsense" in name or "depth camera 415" in name:
                    index = int(cam.index)
                    break
            if index is None:
                if snap.get("health") != "live":
                    rgb.set_error("D415 RGB UVC camera not enumerated yet")
                time.sleep(3)
                continue
            cap = cv2.VideoCapture(index)
            if not cap.isOpened():
                rgb.set_error(f"Could not open D415 RGB at OpenCV index {index}")
                time.sleep(5)
                continue
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            with rgb.lock:
                rgb.backend = "OpenCV AVFoundation"
                rgb.requested_fps = float(cap.get(cv2.CAP_PROP_FPS) or 0)
                rgb.vid = 0x8086
                rgb.pid = 0x0AD3
            failures = 0
            while self.running:
                snap = rgb.snapshot()
                if snap.get("health") == "live" and "pyrealsense2" in (snap.get("backend") or ""):
                    break
                ok, frame = cap.read()
                if not ok or frame is None:
                    failures += 1
                    if failures >= 60:
                        rgb.set_error("D415 RGB opened but is not delivering frames")
                        break
                    time.sleep(0.05)
                    continue
                failures = 0
                jpeg = _encode_bgr(frame)
                if jpeg:
                    h, w = frame.shape[:2]
                    rgb.publish(jpeg, w, h, {"d415_source": f"UVC RGB on :{PORT}"})
            cap.release()
            time.sleep(2)


    def _start_d415_pipeline(self, ctx: Any, serial: str):
        pipeline = rs.pipeline(ctx)
        last_error = "Could not start a D415 profile"
        for attempt in STREAM_ATTEMPTS:
            cfg = rs.config()
            cfg.enable_device(serial)
            color = attempt.get("color")
            depth_size = attempt.get("depth")
            if color:
                w, h, fps = color
                cfg.enable_stream(rs.stream.color, w, h, rs.format.bgr8, fps)
            elif color is None and depth_size is None:
                cfg.enable_stream(rs.stream.color)
                cfg.enable_stream(rs.stream.depth)
            if depth_size:
                w, h, fps = depth_size
                cfg.enable_stream(rs.stream.depth, w, h, rs.format.z16, fps)
            try:
                profile = pipeline.start(cfg)
                print(f"D415 pipeline started {attempt}", flush=True)
                return pipeline, profile, ""
            except Exception as exc:
                last_error = str(exc)
                print(f"D415 pipeline start failed ({serial}) {attempt}: {last_error}", flush=True)
                try:
                    pipeline.stop()
                except Exception:
                    pass
                pipeline = rs.pipeline(ctx)
                continue
        return pipeline, None, last_error

    def _wait_for_d415(self, serial: str, timeout_s: float = 12) -> bool:
        deadline = time.monotonic() + timeout_s
        while self.running and time.monotonic() < deadline:
            try:
                for device in rs.context().query_devices():
                    if device.get_info(rs.camera_info.serial_number) == serial:
                        return True
            except Exception:
                pass
            time.sleep(0.4)
        return False

    def _run_d415_uvc_fallback(self, rgb: StreamState) -> None:
        index = None
        for cam in enumerate_cameras():
            name = (cam.name or "").lower()
            if "realsense" in name or "depth camera 415" in name:
                index = int(cam.index)
                break
        if index is None:
            return
        cap = cv2.VideoCapture(index)
        if not cap.isOpened():
            return
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        try:
            deadline = time.monotonic() + 2.5
            while self.running and time.monotonic() < deadline:
                ok, frame = cap.read()
                if not ok or frame is None:
                    time.sleep(0.05)
                    continue
                jpeg = _encode_bgr(frame)
                if jpeg:
                    h, w = frame.shape[:2]
                    rgb.backend = "OpenCV AVFoundation fallback"
                    rgb.publish(jpeg, w, h, {"d415_source": f"UVC fallback on :{PORT}"})
                    break
        finally:
            cap.release()

    def _try_d415_reset(self, device: Any) -> None:
        now = _now()
        if now - self._d415_last_reset < 20:
            return
        self._d415_last_reset = now
        try:
            serial = device.get_info(rs.camera_info.serial_number)
            print(f"Resetting D415 {serial} after pipeline start failure", flush=True)
            device.hardware_reset()
            time.sleep(4)
        except Exception as exc:
            print(f"D415 hardware reset failed: {exc}", flush=True)

    def _realsense_usb_note(self) -> str:
        with self._system_lock:
            rows = list(self._system.get("usb_devices") or [])
        for row in rows:
            product = (row.get("product") or "").lower()
            if "realsense" in product or "d415" in product:
                speed = usb_speed_label(row.get("device_speed"))
                vid = row.get("vid")
                pid = row.get("pid")
                ids = f"{vid:04X}:{pid:04X}" if vid and pid else "unknown-id"
                return f"USB device {row.get('product')} {ids} at {speed or 'unknown speed'}."
        return "D415 is listed by macOS Camera but not by librealsense in this process."

    def _depth_stats(self, raw: np.ndarray, scale: float, depth_frame: Any) -> dict[str, Any]:
        valid = raw[raw > 0]
        stats: dict[str, Any] = {
            "depth_scale_m": scale,
            "valid_fraction": float((raw > 0).mean()) if raw.size else 0.0,
        }
        if valid.size:
            meters = valid.astype(np.float32) * scale
            stats.update(
                {
                    "min_depth_m": float(meters.min()),
                    "max_depth_m": float(meters.max()),
                    "mean_depth_m": float(meters.mean()),
                    "min_depth_mm": float(meters.min() * 1000),
                    "max_depth_mm": float(meters.max() * 1000),
                    "mean_depth_mm": float(meters.mean() * 1000),
                }
            )
        try:
            video = depth_frame.profile.as_video_stream_profile()
            intr = video.intrinsics
            stats["intrinsics"] = {
                "width": intr.width,
                "height": intr.height,
                "fx": intr.fx,
                "fy": intr.fy,
                "ppx": intr.ppx,
                "ppy": intr.ppy,
                "model": str(intr.model),
                "coeffs": list(intr.coeffs),
            }
            stats["stream_fps"] = video.fps()
        except Exception:
            pass
        return stats

    @staticmethod
    def _colorize_numpy(raw: np.ndarray) -> np.ndarray:
        depth = raw.astype(np.float32)
        valid = depth > 0
        color = np.zeros((raw.shape[0], raw.shape[1], 3), dtype=np.uint8)
        if valid.any():
            lo = float(np.percentile(depth[valid], 2))
            hi = float(np.percentile(depth[valid], 98))
            span = max(hi - lo, 1.0)
            norm = np.zeros_like(depth, dtype=np.uint8)
            norm[valid] = np.clip((depth[valid] - lo) / span * 255.0, 0, 255).astype(np.uint8)
            color = cv2.applyColorMap(norm, cv2.COLORMAP_TURBO)
            color[~valid] = 0
        return color

    def _system_loop(self) -> None:
        while self.running:
            time.sleep(8)
            try:
                self.refresh_system()
            except Exception:
                pass


hub = CameraHub()
app = FastAPI(title="Camera Dashboard", docs_url=None, redoc_url=None)
app.mount("/static", StaticFiles(directory=STATIC), name="static")
NO_CACHE = {"Cache-Control": "no-store, no-cache, must-revalidate, max-age=0", "Pragma": "no-cache"}


@app.middleware("http")
async def _no_cache(request, call_next):
    response = await call_next(request)
    for key, value in NO_CACHE.items():
        response.headers[key] = value
    return response


@app.on_event("startup")
def _startup() -> None:
    hub.start()
    print(f"\nCamera dashboard: http://{HOST}:{PORT}\n", flush=True)


@app.on_event("shutdown")
def _shutdown() -> None:
    hub.stop()


@app.get("/")
def index() -> FileResponse:
    return FileResponse(STATIC / "index.html", headers=NO_CACHE)


@app.get("/api/system")
def api_system() -> JSONResponse:
    return JSONResponse(hub.system_snapshot())


@app.get("/api/streams")
def api_streams() -> JSONResponse:
    return JSONResponse({"streams": hub.list_streams(), "unix": _now()})


@app.post("/api/rescan")
def api_rescan() -> JSONResponse:
    return JSONResponse(hub.refresh_system())


@app.get("/snapshot/{stream_id}")
def snapshot(stream_id: str):
    state = hub.get(stream_id)
    if state is None:
        return JSONResponse({"error": "unknown stream"}, status_code=404, headers=NO_CACHE)
    return Response(content=state.current_jpeg(), media_type="image/jpeg", headers=NO_CACHE)


@app.get("/stream/{stream_id}")
def mjpeg(stream_id: str) -> StreamingResponse:
    state = hub.get(stream_id)
    if state is None:
        return JSONResponse({"error": "unknown stream"}, status_code=404)

    def generate():
        while hub.running:
            jpeg = state.current_jpeg()
            yield b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + jpeg + b"\r\n"
            time.sleep(0.04)

    return StreamingResponse(generate(), media_type="multipart/x-mixed-replace; boundary=frame")


if __name__ == "__main__":
    print(f"Starting camera dashboard on http://{HOST}:{PORT} mode={MODE} uid={os.getuid()}", flush=True)
    uvicorn.run(app, host=HOST, port=PORT, log_level="info")
