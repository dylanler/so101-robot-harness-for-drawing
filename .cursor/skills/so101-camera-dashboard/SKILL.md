---
name: so101-camera-dashboard
description: Starts, checks and debugs the local camera dashboards for the SO-101 drawing rig - :8090 (Logitech C922 + wrist cam, runs as the user) and :8091 (Intel RealSense D415 RGB + depth, runs as root). Use when the user asks to open the camera viewer, when a camera tile shows an error or goes stale, when snapshots fail, or when the D415 / RealSense is not delivering frames.
---

# SO-101 camera dashboards

| URL | Cameras | Process |
| --- | --- | --- |
| http://127.0.0.1:8090 | C922 (1280x720) + wrist cam (640x480), MJPG 30 fps | `camera_server.py` as the user, `DASH_MODE=uvc` |
| http://127.0.0.1:8091 | D415 RGB + colorized depth, 640x480 @ 15 fps | `camera_server.py` as root, `DASH_MODE=realsense` |

## Start / check

```bash
cd camera_dashboard
./supervise.sh --detach uvc 8090 server.log      # no sudo
sudo ./supervise.sh --detach realsense 8091      # the user must type the password; never guess it
curl -s http://127.0.0.1:8090/api/streams | python3 -m json.tool | grep -E '"id"|health|fps'
curl -s http://127.0.0.1:8091/api/streams | python3 -m json.tool | grep -E '"id"|health|fps'
open http://127.0.0.1:8090 http://127.0.0.1:8091
```

Healthy = every stream `"health": "live"` with a rising frame count. Stills:
`python scripts/so101_sketch.py snap --tag check` → `work/live/check-*.jpg`.

## Debug table

| Symptom | Cause | Fix |
| --- | --- | --- |
| `no-uvc` tile / "C922 / USB2.0_CAM1 were not enumerated" | Cameras unplugged (check `system_profiler SPCameraDataType`) | Ask the user to plug in the USB hub |
| D415 `failed to set power state`, `RS2_USB_STATUS_ACCESS`, "0 RealSense devices" | macOS UVC driver owns the D415; librealsense needs root | Run the realsense instance with sudo; never open the D415 via OpenCV/AVFoundation |
| Webcams fail with "camera access denied" in a root process | macOS Camera permission is per user | Webcams only in the user instance (:8090) |
| Wrist cam goes `stale`, frame count frozen | USB 2 bandwidth (C922 720p + D415) | Keep MJPG at 30 fps and the per-camera sizes in `KNOWN_DEVICES` |
| `address already in use` loop in the log | Orphaned server still holds the port | `lsof -nP -iTCP:8090 -sTCP:LISTEN`, kill that PID; the supervisor rebinds |
| Server dies with the agent's shell | Not detached | Always `supervise.sh --detach` |

## Process hygiene

* Stop by exact command: `pkill -f "supervise.sh uvc"` then kill the listener PID from `lsof`.
  The framework Python shows up as `Python camera_server.py` (capital P), so `pkill -f "python …"`
  misses it, and loose patterns can kill the supervisor instead of the server.
* Root processes can only be stopped with sudo (`sudo pkill -f "supervise.sh realsense"`).
* Optional single page: `D415_PROXY=http://127.0.0.1:8091 ./supervise.sh --detach uvc 8090 server.log`
  mirrors the D415 tiles into :8090.

Background and history: [docs/CAMERAS.md](../../../docs/CAMERAS.md)
