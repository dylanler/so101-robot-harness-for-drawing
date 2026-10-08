#!/bin/bash
# Restart-loop supervisor for the camera dashboard (camera_server.py).
#
# Usage:
#   ./supervise.sh [--detach] <mode> <port> [logfile]
#     mode : auto | all | realsense | uvc   (DASH_MODE, see camera_server.py)
#     port : TCP port to serve on           (DASH_PORT)
#     log  : defaults to server-<port>.log in this directory
#
# macOS needs two instances because the two camera families need different privileges:
#        ./supervise.sh --detach uvc       8090 server.log   # C922 + wrist cam: Camera permission
#                                                            # is per user, root has none. No sudo.
#   sudo ./supervise.sh --detach realsense 8091              # D415 depth + RGB: librealsense can
#                                                            # only claim the USB interfaces as root
# Extra env vars pass through, e.g. D415_PROXY=http://127.0.0.1:8091 ./supervise.sh --detach uvc 8090
# mirrors the D415 tiles into the user-level page as well.
# --detach re-launches this script in its own session (so it survives the launching shell,
# including the `sudo` / osascript admin helper exiting) and returns immediately.
set -u
cd "$(dirname "$0")" || exit 1

if [ "${1:-}" = "--detach" ]; then
  shift
  pid=$(.venv/bin/python - "$0" "$@" <<'PY'
import subprocess, sys
proc = subprocess.Popen(
    sys.argv[1:],
    start_new_session=True,
    stdin=subprocess.DEVNULL,
    stdout=subprocess.DEVNULL,
    stderr=subprocess.DEVNULL,
)
print(proc.pid)
PY
)
  echo "supervisor started in background: mode=${1:-auto} port=${2:-8090} pid=$pid"
  exit 0
fi

MODE="${1:-auto}"
PORT="${2:-8090}"
LOG="${3:-server-$PORT.log}"

echo "[supervisor] starting mode=$MODE port=$PORT uid=$(id -u) at $(date)" >> "$LOG"
while true; do
  DASH_MODE="$MODE" DASH_PORT="$PORT" .venv/bin/python camera_server.py >> "$LOG" 2>&1
  code=$?
  echo "[supervisor] dashboard mode=$MODE port=$PORT exited code=$code at $(date)" >> "$LOG"
  sleep 2
done
