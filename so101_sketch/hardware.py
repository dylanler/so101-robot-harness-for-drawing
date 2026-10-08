"""Thin wrappers around lerobot's SO101Follower (connect, read joints, send joint targets)."""

from __future__ import annotations

import time

from .config import JOINTS, MAX_RELATIVE_TARGET_DEG, P_GAIN, PORT, ROBOT_ID


def connect(port: str = PORT, robot_id: str = ROBOT_ID, p_gain: int = P_GAIN):
    """Connect without re-calibrating; torque stays on at disconnect so the arm holds its pose.

    Note: lerobot's configure() briefly disables torque while writing servo settings, so the
    arm sags a little on every connect. Keep one connection per job instead of many short ones.
    """
    from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig

    cfg = SO101FollowerConfig(
        port=port,
        id=robot_id,
        cameras={},
        use_degrees=True,
        max_relative_target=MAX_RELATIVE_TARGET_DEG,
        disable_torque_on_disconnect=False,
        position_p_coefficient=p_gain,
    )
    robot = SO101Follower(cfg)
    robot.connect(calibrate=False)
    return robot


def read_q(robot) -> dict[str, float]:
    obs = robot.get_observation()
    return {k: float(obs[f"{k}.pos"]) for k in JOINTS}


def read_raw(robot) -> dict[str, int]:
    return robot.bus.sync_read("Present_Position", normalize=False)


def read_goal(robot) -> dict[str, int]:
    return robot.bus.sync_read("Goal_Position", normalize=False)


def read_load(robot) -> dict[str, int]:
    return robot.bus.sync_read("Present_Load", normalize=False)


def send(robot, q: dict[str, float]) -> None:
    robot.send_action({f"{k}.pos": float(v) for k, v in q.items()})


def move_joint(robot, q_from: dict, q_to: dict, duration: float, hz: float = 40.0) -> dict:
    """Linear joint-space interpolation."""
    n = max(1, int(duration * hz))
    for i in range(1, n + 1):
        a = i / n
        send(robot, {k: (1 - a) * q_from[k] + a * q_to[k] for k in q_to})
        time.sleep(1.0 / hz)
    return dict(q_to)


def fmt(q: dict) -> dict:
    return {k: round(v, 2) for k, v in q.items()}
