"""SO-101 forward kinematics, pen model and numeric IK (numpy only).

The chain is transcribed from ``so101_new_calib.urdf`` (TheRobotStudio SO-ARM100 repo, also
shipped with lerobot). In that URDF 0 deg is the middle of each joint's calibrated range,
which is what lerobot returns with ``use_degrees=True``, so joint readings plug straight in.
"""

from __future__ import annotations

import math

import numpy as np

from .config import IK_JOINTS, LIMITS_DEG


def _rot(rpy):
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return rz @ ry @ rx


def _tf(xyz, rpy):
    t = np.eye(4)
    t[:3, :3] = _rot(rpy)
    t[:3, 3] = xyz
    return t


def _rz(q):
    c, s = math.cos(q), math.sin(q)
    t = np.eye(4)
    t[0, 0], t[0, 1], t[1, 0], t[1, 1] = c, -s, s, c
    return t


# joint name, origin xyz (m), origin rpy (rad) -- all joint axes are local z
CHAIN = (
    ("shoulder_pan", (0.0388353, -8.97657e-09, 0.0624), (math.pi, 0.0, -math.pi)),
    ("shoulder_lift", (-0.0303992, -0.0182778, -0.0542), (-math.pi / 2, -math.pi / 2, 0.0)),
    ("elbow_flex", (-0.11257, -0.028, 0.0), (0.0, 0.0, math.pi / 2)),
    ("wrist_flex", (-0.1349, 0.0052, 0.0), (0.0, 0.0, -math.pi / 2)),
    ("wrist_roll", (0.0, -0.0611, 0.0181), (math.pi / 2, 0.0486795, math.pi)),
)
GRIPPER_FRAME = _tf((-0.0079, -0.000218121, -0.0981274), (0.0, math.pi, 0.0))
_STATIC = [_tf(xyz, rpy) for _, xyz, rpy in CHAIN]


def fk_frames(q_deg: dict[str, float]) -> tuple[np.ndarray, np.ndarray]:
    """Returns (gripper_frame_link pose 4x4, world unit axis of the shoulder_lift joint)."""
    t = np.eye(4)
    lift_axis = None
    for (name, _, _), st in zip(CHAIN, _STATIC):
        t = t @ st
        if name == "shoulder_lift":
            lift_axis = t[:3, 2].copy()
        t = t @ _rz(math.radians(q_deg[name]))
    return t @ GRIPPER_FRAME, lift_axis


def fk(q_deg: dict[str, float]) -> np.ndarray:
    """Pose (4x4, metres) of gripper_frame_link in base_link."""
    return fk_frames(q_deg)[0]


class PenModel:
    """Pen tip = gripper frame origin + L * (the gripper-frame axis that points down).

    The pen axis is chosen once, from a reference pose in which the pen points at the paper
    (the gripper-frame axis best aligned with world -z). This works for any rigid mount: a pen
    taped along the jaws picks one axis, a pen clamped at 90 deg to the jaws picks another.
    """

    def __init__(self, q_ref: dict[str, float], pen_len_m: float, pitch_rad: float | None = None):
        t0 = fk(q_ref)
        r0 = t0[:3, :3]
        dots = [float(r0[:, i] @ np.array([0, 0, -1.0])) for i in range(3)]
        i = int(np.argmax(np.abs(dots)))
        self.axis_local = np.zeros(3)
        self.axis_local[i] = 1.0 if dots[i] > 0 else -1.0
        if pen_len_m <= 0:
            # auto: assume the tip rests on the paper, which lies in the base plane z=0
            axis_w = r0 @ self.axis_local
            pen_len_m = float(t0[2, 3] / max(1e-6, -axis_w[2]))
        self.L = pen_len_m
        self.axis_world0 = r0 @ self.axis_local
        self.q_ref = dict(q_ref)
        self.pitch0 = self.pitch(q_ref) if pitch_rad is None else pitch_rad

    def tip(self, q: dict[str, float]) -> np.ndarray:
        t = fk(q)
        return t[:3, 3] + self.L * (t[:3, :3] @ self.axis_local)

    def pitch(self, q: dict[str, float]) -> float:
        """Signed angle of the pen axis from vertical, measured in the arm's pitch plane.

        The three pitch joints share the shoulder_lift axis direction n, so the in-plane
        horizontal direction h = n x ez rotates with the pan and this scalar is pan-invariant.
        """
        t, n = fk_frames(q)
        d = t[:3, :3] @ self.axis_local
        h = np.cross(n, np.array([0.0, 0.0, 1.0]))
        h /= np.linalg.norm(h)
        return math.atan2(float(d @ h), float(-d[2]))

    def _residual(self, q, target, pitch, w_pitch=0.03):
        r = list(self.tip(q) - target)  # metres, weight 1
        # soft preference: keep the pen attitude (1 rad off == w_pitch metres of position error)
        r.append(w_pitch * (self.pitch(q) - pitch))
        # joint limits as hinge penalties (1 deg over == 5 mm of position error)
        for name in IK_JOINTS:
            lo, hi = LIMITS_DEG[name]
            r.append(0.005 * (max(0.0, q[name] - hi) + max(0.0, lo - q[name])))
        return np.array(r)

    def ik(self, q_seed: dict[str, float], target: np.ndarray, pitch: float | None = None, iters: int = 60):
        """Damped least squares on (pan, lift, elbow, wrist_flex).

        Position is the hard goal; pen pitch is a soft preference that yields when a joint
        would leave its calibrated range. Returns (q, err_m, pitch_rad).
        """
        q = dict(q_seed)
        pitch = self.pitch0 if pitch is None else pitch
        # pass 1: strong attitude preference picks the posture; pass 2: nearly free attitude nails position
        for w_pitch, n_it in ((0.03, iters), (0.002, iters // 2)):
            for _ in range(n_it):
                r = self._residual(q, target, pitch, w_pitch)
                if np.linalg.norm(r[:3]) < 3e-5 and np.linalg.norm(r[4:]) < 1e-6:
                    break
                jac = np.zeros((len(r), 4))
                h = 1e-3  # degrees
                for j, name in enumerate(IK_JOINTS):
                    qh = dict(q)
                    qh[name] += h
                    jac[:, j] = (self._residual(qh, target, pitch, w_pitch) - r) / h
                lam = 1e-7
                dq = -np.linalg.solve(jac.T @ jac + lam * np.eye(4), jac.T @ r)
                dq = np.clip(dq, -6.0, 6.0)
                for j, name in enumerate(IK_JOINTS):
                    q[name] = float(q[name] + dq[j])
        for name in IK_JOINTS:  # hard clip as last resort
            q[name] = float(np.clip(q[name], *LIMITS_DEG[name]))
        err = float(np.linalg.norm(self.tip(q) - target))
        return q, err, self.pitch(q)
