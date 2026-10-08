"""Pen-plotting controller for the SO-101: homing, Cartesian moves, paper calibration, drawing.

``Sketcher(robot=None, ...)`` runs as a *dry run*: identical IK and motion planning, but no
servo commands and no sleeping, so a design can be checked for reachability and timed offline.
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass, field

import numpy as np

from .config import LIMITS_DEG, PITCH_JOINTS
from .designs.geometry import resample
from .hardware import fmt, move_joint, read_goal, read_load, read_q, read_raw, send
from .kinematics import PenModel
from .setup_profile import PaperSurface, SetupProfile

IK_TOL_M = 1.5e-3


@dataclass
class RunStats:
    """What a (dry or real) run did: used for dry-run reports and tool-call results."""

    strokes: int = 0
    ink_mm: float = 0.0
    travel_mm: float = 0.0
    commands: int = 0
    seconds: float = 0.0
    max_ik_err_mm: float = 0.0
    pitch_deg: list[float] = field(default_factory=lambda: [math.inf, -math.inf])
    failures: list[str] = field(default_factory=list)

    def as_dict(self) -> dict:
        return {
            "strokes": self.strokes,
            "ink_mm": round(self.ink_mm, 1),
            "travel_mm": round(self.travel_mm, 1),
            "servo_commands": self.commands,
            "est_minutes": round(self.seconds / 60.0, 2),
            "max_ik_err_mm": round(self.max_ik_err_mm, 2),
            "pen_pitch_range_deg": [round(p, 1) for p in self.pitch_deg] if self.commands else None,
            "failures": self.failures[:10],
            "n_failures": len(self.failures),
        }


class Sketcher:
    # ---- servo-lag contact probe tuning (stiff pen mounts only; see docs/CALIBRATION.md)
    PROBE_HOVER = 0.012  # start the descent this far above the expected contact
    PROBE_SKIP = 0.005  # ignore the direction-change transient (servo stiction) at the top
    PROBE_WINDOW = 0.003  # regression window for the slope test

    def __init__(self, robot, profile: SetupProfile, q_start: dict[str, float] | None = None, hz: float = 40.0):
        self.robot = robot
        self.dry = robot is None
        self.prof = profile
        self.q = read_q(robot) if robot is not None else dict(q_start or {**profile.pen_reference_q, **profile.home_seed})
        self.fixed = {"wrist_roll": self.q["wrist_roll"], "gripper": self.q["gripper"]}
        self.pen = PenModel(profile.pen_reference_q, profile.pen_length_mm / 1000.0, profile.pitch_rad)
        self.hz = hz
        self.stats = RunStats()
        self.tip0 = self.pen.tip(self.q)

    # ---- low level -------------------------------------------------------------------------
    def _send(self, q: dict) -> None:
        self.stats.commands += 1
        p = math.degrees(self.pen.pitch(q))
        self.stats.pitch_deg = [min(self.stats.pitch_deg[0], p), max(self.stats.pitch_deg[1], p)]
        if not self.dry:
            send(self.robot, q)

    def _sleep(self, s: float) -> None:
        self.stats.seconds += s
        if not self.dry:
            time.sleep(s)

    def solve(self, xy, z, seed=None) -> dict:
        target = np.array([xy[0], xy[1], z])
        q, err, _ = self.pen.ik(seed or self.q, target)
        if err > IK_TOL_M:
            raise RuntimeError(f"IK failed err={err * 1000:.1f}mm target={np.round(target * 1000, 1)} q={fmt(q)}")
        self.stats.max_ik_err_mm = max(self.stats.max_ik_err_mm, err * 1000)
        q.update(self.fixed)
        return q

    def tip(self) -> np.ndarray:
        return self.pen.tip(self.q)

    def surface_z(self, xy) -> float:
        return self.prof.surface_z(xy) if self.prof.surface else float(self.tip0[2])

    # ---- motion ----------------------------------------------------------------------------
    def goto_xyz(self, xy, z, speed: float | None = None) -> None:
        """Straight tip-space line at ``speed`` m/s, IK warm-started point to point."""
        speed = speed or self.prof.speed_mm_s / 1000.0
        target = np.array([xy[0], xy[1], z])
        cur = self.pen.tip(self.q)
        dist = float(np.linalg.norm(target - cur))
        n = max(2, int(dist / speed * self.hz))
        for i in range(1, n + 1):
            p = cur + (target - cur) * (i / n)
            self.q = self.solve(p[:2], p[2])
            self._send(self.q)
            self._sleep(1.0 / self.hz)

    def home(self, height: float = 0.03) -> None:
        """Joint-space move onto the drawing posture above the centre (safe from any parked pose).

        Cartesian moves from a parked pose can drag the IK onto a different elbow branch, so the
        first move is in joint space, seeded from the profile's known-good posture.
        """
        seed = dict(self.q)
        if seed["shoulder_lift"] < -30.0:  # parked high/back
            seed.update(self.prof.home_seed)
        zc = self.surface_z(self.prof.centre)
        q_home = self.solve(self.prof.centre, zc + height, seed=seed)

        def lerp(a, b, t):
            return {k: (1 - t) * a[k] + t * b[k] for k in b}

        # make sure the straight joint-space path does not dip into the paper
        lo = min(self.pen.tip(lerp(self.q, q_home, t))[2] for t in np.linspace(0, 1, 40))
        if lo < zc + 0.005:
            q_up = dict(self.q)  # raise the upper arm first (more negative lift = arm swings up/back)
            q_up["shoulder_lift"] = max(LIMITS_DEG["shoulder_lift"][0], q_up["shoulder_lift"] - 12.0)
            self._joint_move(q_up, 1.5)
        self._joint_move(q_home, 2.5)
        self._sleep(0.3)
        print(f"home posture q={fmt(self.q)} pitch {math.degrees(self.pen.pitch(self.q)):+.1f} deg", flush=True)

    def _joint_move(self, q_to: dict, duration: float) -> None:
        if self.dry:
            self.stats.seconds += duration
            self.q = dict(q_to)
        else:
            self.q = move_joint(self.robot, self.q, q_to, duration)

    def park(self, back_mm: float = 70.0, up_mm: float = 45.0) -> None:
        """Lift and pull back toward the robot so the C922 sees the whole drawing."""
        xy = self.tip()[:2]
        for up in (up_mm, 30.0, 15.0):  # some setups cannot lift far near the robot
            try:
                self.goto_xyz(xy, self.surface_z(xy) + up / 1000.0, speed=0.04)
                break
            except RuntimeError:
                continue
        c = self.prof.centre + self.prof.v_dir * back_mm / 1000.0
        try:
            self.goto_xyz(c, self.tip()[2], speed=0.05)
        except RuntimeError as exc:
            print(f"park: stayed above the last stroke ({exc.args[0][:60]})", flush=True)

    # ---- reachability (no motion) ----------------------------------------------------------
    def scan(self, nu: int = 7, nv: int = 9) -> int:
        """Print a reachability table over the drawing rectangle at paper height."""
        bad = 0
        for v in np.linspace(0, 1, nv)[::-1]:
            row = []
            for u in np.linspace(0, 1, nu):
                xy = self.prof.paper_xy(u, v)
                z = self.surface_z(xy)
                q, err, pitch = self.pen.ik({**self.q, **self.prof.home_seed}, np.array([xy[0], xy[1], z]))
                ok = err < IK_TOL_M
                bad += not ok
                row.append(f"{'ok' if ok else 'XX'}{math.degrees(pitch - self.pen.pitch0):+5.1f}")
            print(f"v={v:.2f} | " + "  ".join(row))
        print(f"{bad} unreachable of {nu * nv}; cells show pen pitch change (deg) vs the profile's pitch")
        return bad

    # ---- drawing ---------------------------------------------------------------------------
    def draw(self, strokes: list[np.ndarray], on_stroke=None) -> RunStats:
        """Draw normalised (u, v) strokes inside the profile's drawing rectangle."""
        p = self.prof
        print(f"drawing {len(strokes)} strokes, area {p.area_mm[0]:.0f}x{p.area_mm[1]:.0f}mm", flush=True)
        for si, s in enumerate(strokes):
            xy_path = resample(np.array([p.paper_xy(u, v) for u, v in s]), p.step_mm / 1000.0)
            surf = np.array([self.surface_z(xy) for xy in xy_path])
            z_up = surf + p.hover_mm / 1000.0
            z_dn = surf - p.press_mm / 1000.0
            try:
                start = self.tip()
                self.goto_xyz(xy_path[0], z_up[0], speed=0.08)  # travel
                self.stats.travel_mm += float(np.linalg.norm(self.tip() - start)) * 1000
                self.goto_xyz(xy_path[0], z_dn[0], speed=0.02)  # pen down
                self._sleep(0.05)
                for xy, z in zip(xy_path[1:], z_dn[1:]):
                    self.q = self.solve(xy, z)
                    self._send(self.q)
                    self._sleep(p.step_mm / p.speed_mm_s)
                self.goto_xyz(xy_path[-1], z_up[-1], speed=0.03)  # pen up
            except RuntimeError as exc:
                if not self.dry:
                    raise
                self.stats.failures.append(f"stroke {si}: {exc}")
                continue
            self.stats.strokes += 1
            self.stats.ink_mm += float(np.sum(np.linalg.norm(np.diff(xy_path, axis=0), axis=1))) * 1000
            if not self.dry:
                print(f"stroke {si + 1}/{len(strokes)} done ({len(xy_path)} pts)", flush=True)
            if on_stroke is not None:
                on_stroke(si, len(strokes))
        return self.stats

    # ---- paper calibration: ink ladder -----------------------------------------------------
    def ladder(self, u0: float, vs: tuple, levels_mm: tuple, dash_mm: float = 12.0, gap_mm: float = 7.0) -> list[dict]:
        """At each v, draw one short dash per commanded height (absolute model z, mm).

        Read the camera afterwards: the highest level that still inks is the paper height for
        that reach. Dashes stack ``gap_mm`` apart along v; travel happens 30 mm above each level
        so the pen cannot drag between dashes even if the surface is far above the guess.
        """
        w, h = self.prof.area_m
        marks = []
        for v in vs:
            for k, zmm in enumerate(levels_mm):
                vv = v + k * gap_mm / 1000.0 / h
                a = self.prof.paper_xy(u0, vv)
                b = self.prof.paper_xy(u0 + dash_mm / 1000.0 / w, vv)
                z = zmm / 1000.0
                self.goto_xyz(a, z + 0.030, speed=0.05)
                self.goto_xyz(a, z, speed=0.03)
                self.goto_xyz(b, z, speed=0.02)
                self.goto_xyz(b, z + 0.030, speed=0.04)
                marks.append({"v": round(vv, 3), "x": float(a[0]), "y": float(a[1]), "z_mm": zmm})
                print(f"  ladder v={v:.2f} level {k}: z={zmm:+.1f} mm at x={a[0] * 1000:.0f} mm", flush=True)
        self.goto_xyz(self.prof.centre, 0.05, speed=0.05)
        return marks

    # ---- paper calibration: servo-lag contact probe ---------------------------------------
    def contact_signal(self):
        """Position error (present - goal ticks) of the pitch servos + their raw loads."""
        raw, load, goal = read_raw(self.robot), read_load(self.robot), read_goal(self.robot)
        err = {k: raw[k] - goal[k] for k in PITCH_JOINTS}
        return err, {k: load[k] for k in PITCH_JOINTS}

    def _down_signs(self, xy, z) -> dict[str, float]:
        """Per joint: +1/-1 if its tick count grows/shrinks when the tip moves down (0 if unaffected)."""
        qa = self.solve(xy, z)
        qb = self.solve(xy, z - 0.003, seed=qa)
        signs = {}
        for j in PITCH_JOINTS:
            d = (qb[j] - qa[j]) * 4095 / 360  # ticks per 3 mm
            signs[j] = float(np.sign(d)) if abs(d) > 0.6 else 0.0
        return signs

    def _lag(self, signs) -> tuple[float, dict]:
        """Sum over pitch joints of (present-goal) ticks projected on the 'down' direction.

        Free air: roughly constant (gravity sag). Pen pressed on paper: the goal keeps going down
        but the joints cannot follow, so the metric drops (becomes more negative).
        """
        err, load = self.contact_signal()
        return float(sum(err[j] * signs[j] for j in PITCH_JOINTS)), {"err": err, "load": load}

    def _settle(self, signs, timeout=2.5) -> float:
        t0 = time.time()
        hist = []
        while time.time() - t0 < timeout:
            m, _ = self._lag(signs)
            hist.append(m)
            if len(hist) >= 6 and max(hist[-5:]) - min(hist[-5:]) <= 1.0 and time.time() - t0 > 0.8:
                break
            time.sleep(0.08)
        return float(np.mean(hist[-5:]))

    def probe_point(self, xy, z_guess, log=None, max_depth=0.015, detect=True, hover=None, speed_mm_s=2.0) -> float:
        """Lower the pen at xy until contact; returns the commanded tip z where it met the paper (nan if not).

        Slow continuous ramp. After the stiction transient (first ~5 mm) the lag metric is flat in
        free air; once the pen is blocked the commanded joints run away from the measured ones.
        Contact = lag over the last 3 mm slopes steeper than 1.8 ticks/mm and has dropped >= 9
        ticks from its running max, confirmed by >= 5 more ticks over the next 3 mm (stiction
        fades, real contact keeps building). The knee is extrapolated back along the slope.
        """
        if self.dry:
            raise RuntimeError("probing needs the robot")
        hover0 = self.PROBE_HOVER if hover is None else hover
        start = z_guess + hover0
        for hov in (hover0, 0.009, 0.007):  # near the robot the arm cannot reach high: start lower
            start = z_guess + hov
            try:
                self.goto_xyz(xy, start, speed=0.04)
                break
            except RuntimeError as exc:
                if hov == 0.007:
                    raise
                print(f"  cannot hover {hov * 1000:.0f} mm above ({exc.args[0][:40]}); trying lower", flush=True)
        signs = self._down_signs(xy, z_guess)
        self._settle(signs)
        z = start
        dz = speed_mm_s / 1000.0 / 25.0  # per cycle @ 25 Hz
        zs: list[float] = []
        lags: list[float] = []
        run_max = None
        contact_z = None
        candidate = None  # (z_detect, lag_detect, contact_estimate)
        while z > z_guess - max_depth:
            z -= dz
            self.q = self.solve(xy, z)
            send(self.robot, self.q)
            time.sleep(0.04)
            m, info = self._lag(signs)
            fallen = start - z
            if fallen >= self.PROBE_SKIP:
                zs.append(z)
                lags.append(m)
                run_max = m if run_max is None else max(run_max, m)
            d = (m - run_max) if run_max is not None else 0.0
            if log is not None:
                log.append({"z_mm": round(z * 1000, 2), "lag": round(d, 1), "raw": round(m, 1), **info})
            if not detect or run_max is None or zs[0] - z < self.PROBE_WINDOW:
                continue
            lw = np.array(lags)
            if candidate is None:
                zw = np.array(zs)
                sel = zw >= z - self.PROBE_WINDOW
                if sel.sum() >= 12 and fallen >= 0.007:
                    slope, _ = np.polyfit(zw[sel] * 1000, lw[sel], 1)  # ticks per mm
                    drop = run_max - float(np.median(lw[-3:]))
                    if slope > 1.8 and drop >= 9.0:
                        est = z + min(0.006, max(0.0, (drop - 2.0) / slope / 1000.0))
                        candidate = (z, float(np.median(lw[-3:])), est)
            elif candidate[0] - z >= 0.003:
                extra = candidate[1] - float(np.median(lw[-3:]))
                if extra >= 5.0:
                    contact_z = candidate[2]
                    break
                print(f"  contact candidate at {candidate[2] * 1000:.1f} mm rejected (extra drop {extra:.0f})", flush=True)
                candidate = None
                run_max = float(np.max(lw[-10:]))
        if detect and contact_z is None:
            print(f"  no contact found at xy={np.round(np.asarray(xy) * 1000, 1)} down to {z * 1000:.1f}mm", flush=True)
        self.goto_xyz(xy, (contact_z if contact_z is not None else z_guess) + self.prof.hover_mm / 1000.0, speed=0.03)
        return contact_z if contact_z is not None else float("nan")

    def probe_grid(self, n: int = 3) -> tuple[PaperSurface, np.ndarray]:
        """Centre first, then an n x n grid spiralling out; fits a plane/quadratic, tolerant of misses."""
        pts: list[list[float]] = []
        first_depth = 0.045
        if self.prof.surface is not None:
            z_guess = self.prof.surface_z(self.prof.centre)
            first_depth = 0.03
            print(f"probe: starting from previous surface, centre contact ~{z_guess * 1000:.1f} mm", flush=True)
        else:
            z_guess = float(self.tip0[2])
        c = self.prof.centre

        def predict(xy):
            if len(pts) >= 3:
                a = np.array(pts)
                b = np.stack([np.ones(len(a)), a[:, 0] - c[0], a[:, 1] - c[1]], axis=1)
                coef, *_ = np.linalg.lstsq(b, a[:, 2], rcond=None)
                return float(coef @ [1.0, xy[0] - c[0], xy[1] - c[1]])
            return float(pts[-1][2]) if pts else z_guess

        grid = [(u, v) for v in np.linspace(0.12, 0.88, n) for u in np.linspace(0.12, 0.88, n)]
        grid.sort(key=lambda uv: (abs(uv[1] - 0.5), abs(uv[0] - 0.5)))
        order = [(0.5, 0.5)] + grid
        for k, (u, v) in enumerate(order):
            xy = self.prof.paper_xy(u, v)
            guess = predict(xy)
            try:
                if k == 0 and self.prof.surface is None:
                    zc = self.probe_point(xy, guess, max_depth=first_depth, hover=0.03, speed_mm_s=4.0)
                else:
                    zc = self.probe_point(xy, guess, max_depth=first_depth if k == 0 else 0.015)
                if not math.isnan(zc) and len(pts) >= 3 and abs(zc - guess) > 0.006:
                    print(f"  outlier ({zc * 1000:.1f} vs fit {guess * 1000:.1f} mm), re-probing deeper", flush=True)
                    zc = self.probe_point(xy, guess, max_depth=0.02)
                    if not math.isnan(zc) and abs(zc - guess) > 0.006:
                        zc = float("nan")
            except RuntimeError as exc:
                print(f"  probe skipped: {exc}", flush=True)
                zc = float("nan")
            ok = not math.isnan(zc)
            if ok:
                pts.append([float(xy[0]), float(xy[1]), float(zc)])
            print(f"probe {k + 1}/{len(order)} uv=({u:.2f},{v:.2f}) contact z={'%.2f' % (zc * 1000) if ok else 'miss'} mm", flush=True)
        if len(pts) < 3:
            raise RuntimeError(f"only {len(pts)} contact points; cannot fit the paper surface")
        return PaperSurface.fit(pts, float(c[0]), float(c[1]), "servo-lag probe grid")
