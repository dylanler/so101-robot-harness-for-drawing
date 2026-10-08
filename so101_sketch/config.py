"""Hardware constants and repository paths shared by every module."""

from __future__ import annotations

import os
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
WORK = REPO / "work"  # runtime output (snapshots, previews, ladder logs); git-ignored
LIVE = WORK / "live"
SETUPS = REPO / "calibration" / "setups"
DEFAULT_SETUP = SETUPS / "pen90_letter_portrait.json"

# --- robot ------------------------------------------------------------------------------
PORT = os.environ.get("SO101_PORT", "/dev/cu.usbmodem5A7A0545771")
ROBOT_ID = os.environ.get("SO101_ID", "so101_sketch_follower")
JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper")
IK_JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex")
PITCH_JOINTS = ("shoulder_lift", "elbow_flex", "wrist_flex")
# Feetech position P gain. lerobot's default (16) sags several mm under gravity; 32 halves it
# and gives a cleaner contact signal.
P_GAIN = 32
MAX_RELATIVE_TARGET_DEG = 12.0

# Calibrated raw tick ranges (calibration/lerobot_so101_sketch_follower.json). With
# use_degrees=True lerobot reports 0 deg at the middle of each range, which is exactly the
# zero of the so101_new_calib URDF.
CAL_TICKS = {
    "shoulder_pan": (765, 3437),
    "shoulder_lift": (914, 3280),
    "elbow_flex": (820, 3027),
    "wrist_flex": (920, 3230),
}
LIMIT_MARGIN_DEG = 4.0
LIMITS_DEG = {
    k: (-(hi - lo) / 2 * 360 / 4095 + LIMIT_MARGIN_DEG, (hi - lo) / 2 * 360 / 4095 - LIMIT_MARGIN_DEG)
    for k, (lo, hi) in CAL_TICKS.items()
}

# --- cameras ----------------------------------------------------------------------------
# :8090 = webcams (runs as the logged-in user), :8091 = RealSense D415 (runs as root).
DASHBOARD = os.environ.get("DASHBOARD", "http://127.0.0.1:8090")
DEPTH_DASHBOARD = os.environ.get("DEPTH_DASHBOARD", "http://127.0.0.1:8091")
CAMS = {"wrist": "usb2-0-cam1-05a39230", "c922": "c922-pro-stream-webcam-046d085c"}
DEPTH_CAMS = {"d415_rgb": "realsense-rgb", "d415_depth": "realsense-depth"}
# OpenCV AVFoundation indices, used only when the dashboard is down.
CAM_INDEX = {"wrist": 0, "c922": 2}
