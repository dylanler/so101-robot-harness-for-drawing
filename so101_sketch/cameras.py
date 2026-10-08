"""Camera stills: from the local camera dashboards, falling back to OpenCV directly."""

from __future__ import annotations

import time
import urllib.request
from pathlib import Path

from .config import CAM_INDEX, CAMS, DASHBOARD, DEPTH_CAMS, DEPTH_DASHBOARD, LIVE


def snapshot(name: str, tag: str, out_dir: Path = LIVE) -> Path | None:
    """Save one still as ``<out_dir>/<tag>-<name>.jpg``.

    ``name`` is ``wrist`` / ``c922`` (webcam dashboard on :8090) or ``d415_rgb`` / ``d415_depth``
    (root dashboard on :8091). Never raises: cameras must not block the robot.
    """
    out_dir.mkdir(parents=True, exist_ok=True)
    out = out_dir / f"{tag}-{name}.jpg"
    if name in DEPTH_CAMS:
        url = f"{DEPTH_DASHBOARD}/snapshot/{DEPTH_CAMS[name]}"
    else:
        url = f"{DASHBOARD}/snapshot/{CAMS[name]}"
    try:
        out.write_bytes(urllib.request.urlopen(url, timeout=3).read())
        return out
    except Exception:
        pass
    if name not in CAM_INDEX:
        print(f"snapshot {name} failed: dashboard {url} unreachable", flush=True)
        return None
    # Direct capture only works while no dashboard holds the camera.
    try:
        import cv2

        cap = cv2.VideoCapture(CAM_INDEX[name], cv2.CAP_AVFOUNDATION)
        if name == "c922":
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        frame = None
        for _ in range(12):  # let exposure settle
            ok, f = cap.read()
            if ok:
                frame = f
            time.sleep(0.05)
        cap.release()
        if frame is None:
            raise RuntimeError("no frame")
        cv2.imwrite(str(out), frame)
        return out
    except Exception as exc:
        print(f"snapshot {name} failed: {exc}", flush=True)
        return None


def snapshot_all(tag: str, include_depth: bool = True) -> dict[str, Path | None]:
    names = list(CAMS) + (list(DEPTH_CAMS) if include_depth else [])
    return {n: snapshot(n, tag) for n in names}
