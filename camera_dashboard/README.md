# Camera dashboard

Live local viewer for USB cameras, Intel RealSense D415 RGB + depth, and device metadata.

This app lives in `camera_dashboard/` (server: `camera_server.py`) and is the only camera viewer for this project. See `../docs/CAMERAS.md` for the physical camera layout and the debugging history.

## URLs (macOS: two instances)

| URL | Runs as | `DASH_MODE` | Tiles |
| --- | --- | --- | --- |
| **http://127.0.0.1:8090** | logged-in user (no sudo) | `uvc` | Logitech C922 + SO-101 wrist cam (USB2.0_CAM1) |
| **http://127.0.0.1:8091** | root (`sudo`) | `realsense` | RealSense D415 RGB + colorized depth |

:8090 is the main viewer and never needs sudo; :8091 is the optional depth add-on.
Both are localhost only. Why two processes: on recent macOS (tested on 26.x, Apple Silicon)
`librealsense` can only claim the D415's USB interfaces as root — as a normal user every
attempt fails with `failed to set power state` / `RS2_USB_STATUS_ACCESS`. But macOS Camera
permission is granted per user, so a root process gets `camera access has been denied` from
AVFoundation and cannot open the C922 / wrist cam. Each family therefore runs under the
privilege level that can actually see it. The mode is auto-detected (`DASH_MODE=auto`:
root → `realsense`, non-root on macOS → `uvc`); `all` forces the legacy single-process mode.

## Run

```bash
cd camera_dashboard
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt

./supervise.sh --detach uvc 8090 server.log              # C922 + wrist cam, no sudo
sudo ./supervise.sh --detach realsense 8091              # D415 depth + RGB, asks for your password
                                                         # (logs to server-8091.log)
```

`supervise.sh` restarts `camera_server.py` if it exits and detaches into its own session. To run a
single instance by hand: `DASH_MODE=uvc DASH_PORT=8090 python camera_server.py`.

Optional: `D415_PROXY=http://127.0.0.1:8091 ./supervise.sh --detach uvc 8090 server.log` makes the
:8090 page also mirror the D415 tiles from the root instance (still no sudo for :8090 itself).

Stop them with `pkill -f "supervise.sh uvc"` / `sudo pkill -f "supervise.sh realsense"`,
then `pkill -f "camera_server.py"` (`sudo` for the root one).

Notes:

- The D415 currently enumerates as a USB 2.1 device (`usb_type` in `/api/streams`), so the SDK
  runs the 640×480 @ 15 fps colour+depth profile; a USB 3 cable/port would allow higher modes.
- The `uvc` instance never imports device lists from `pyrealsense2`, so it does not contend with
  the root instance for the camera.
- Webcams are opened as MJPG at 30 fps with a per-camera size (`KNOWN_DEVICES` in
  `camera_server.py`: C922 1280×720, wrist cam 640×480). Both webcams and the D415 share one USB 2
  bus; uncompressed 720p60 from the C922 alone starves the wrist cam, which then freezes a few
  seconds after opening.

## What you should see

- :8090 — C922 and USB2.0_CAM1 live RGB tiles
- :8091 — D415 RGB and colorized depth from a local `pyrealsense2` pipeline in that process
- Per-stream metadata: device id, resolution, fps, backend, USB IDs, health, last frame age, depth min/mean/max mm and intrinsics
- FaceTime HD (laptop built-in) is omitted on purpose

If a camera cannot stream, that tile shows a live error. The page does not invent frames.

## API

- `GET /api/system`
- `GET /api/streams`
- `GET /snapshot/{id}` — latest JPEG (used by `so101_sketch/cameras.py`)
- `GET /stream/{id}` — MJPEG
- `POST /api/rescan`
