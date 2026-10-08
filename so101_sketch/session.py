"""High-level API: one object per drawing job. Designed to be called from scripts or agent tools.

Offline (no robot needed)::

    from so101_sketch import preview, dry_run
    preview("petronas")                      # -> work/live/petronas-preview.png + stroke stats
    dry_run("references/my_photo.jpg")       # IK-check + time estimate of a traced image

On the robot::

    from so101_sketch import SketchSession
    with SketchSession() as s:               # default: calibration/setups/pen90_letter_portrait.json
        s.draw("petronas")                   # home -> draw -> park -> camera snapshots
"""

from __future__ import annotations

import json
import math
import time
from pathlib import Path

import numpy as np

from .cameras import snapshot, snapshot_all
from .config import DEFAULT_SETUP, LIVE, SETUPS, WORK
from .designs import get_design, render_preview, stroke_stats
from .hardware import connect, fmt, read_load, read_q
from .kinematics import PenModel, fk
from .setup_profile import PaperSurface, SetupProfile
from .sketcher import Sketcher


def _profile(setup) -> SetupProfile:
    return setup if isinstance(setup, SetupProfile) else SetupProfile.load(setup or DEFAULT_SETUP)


# ----------------------------------------------------------------------------- offline tools


def preview(design: str, setup=None, out: Path | None = None, **trace_kw) -> dict:
    """Render a design to PNG at the profile's drawing-rectangle aspect. No robot needed."""
    prof = _profile(setup)
    strokes = get_design(design, tuple(prof.area_mm), **trace_kw)
    name = Path(design).stem if ("/" in design or "." in design) else design
    out = out or LIVE / f"{name}-preview.png"
    render_preview(strokes, out, tuple(prof.area_mm))
    stats = stroke_stats(strokes, tuple(prof.area_mm))
    return {"preview": str(out), "area_mm": prof.area_mm, **{f"{k}_mm" if k in ("ink", "travel") else k: v for k, v in stats.items()}}


def dry_run(design: str, setup=None, step_mm: float | None = None, **trace_kw) -> dict:
    """Plan the whole drawing through the IK without moving the robot.

    Reports unreachable strokes, worst IK error, pen-pitch range and estimated duration.
    ``step_mm`` > profile step speeds this up (e.g. 3) at a small loss of fidelity.
    """
    prof = _profile(setup)
    if step_mm:
        prof.step_mm = step_mm
    strokes = get_design(design, tuple(prof.area_mm), **trace_kw)
    sk = Sketcher(None, prof)
    t0 = time.time()
    sk.home()
    stats = sk.draw(strokes)
    sk.park()
    out = stats.as_dict()
    out["plan_seconds"] = round(time.time() - t0, 1)
    out["ok"] = out["n_failures"] == 0
    return out


def capture_setup(name: str, robot, pen_len_mm: float = 0.0, centre_offset_mm=(0.0, 0.0),
                  area_mm=(130.0, 170.0), description: str = "", base: SetupProfile | None = None) -> SetupProfile:
    """Create a profile from the arm's current pose (pen tip resting on the paper where the drawing centre should be).

    ``pen_len_mm=0`` estimates the pen length assuming the tip touches the table plane (z=0);
    ``centre_offset_mm=(toward_robot, right)`` moves the drawing centre from the pen position.
    The new profile has no paper surface: follow up with ``ladder`` + ``set_surface`` or ``probe``.
    """
    q = read_q(robot)
    pen = PenModel(q, pen_len_mm / 1000.0)
    tip = pen.tip(q)
    c = tip[:2] / np.linalg.norm(tip[:2])
    v_dir, u_dir = -c, np.array([-c[1], c[0]])
    centre = tip[:2] + v_dir * centre_offset_mm[0] / 1000.0 + u_dir * centre_offset_mm[1] / 1000.0
    defaults = base or SetupProfile.load(DEFAULT_SETUP)
    return SetupProfile(
        name=name,
        description=description or f"captured {time.strftime('%Y-%m-%d %H:%M')}",
        pen_reference_q={k: round(v, 2) for k, v in q.items()},
        pen_length_mm=round(pen.L * 1000, 2),
        pen_pitch_deg=round(math.degrees(pen.pitch0), 2),
        home_seed={k: round(q[k], 2) for k in ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex")},
        centre_xy_m=[round(float(centre[0]), 4), round(float(centre[1]), 4)],
        area_mm=list(area_mm),
        press_mm=defaults.press_mm,
        hover_mm=defaults.hover_mm,
        speed_mm_s=defaults.speed_mm_s,
        step_mm=defaults.step_mm,
        surface=None,
        surface_method=defaults.surface_method,
    )


# ----------------------------------------------------------------------------- robot session


class SketchSession:
    """Owns one robot connection and one setup profile (re-connecting sags the arm slightly)."""

    def __init__(self, setup: str | Path | SetupProfile | None = None, robot=None):
        self.setup_path = None if isinstance(setup, SetupProfile) else Path(setup or DEFAULT_SETUP)
        self.prof = _profile(setup)
        self.robot = robot or connect()
        self.sk = Sketcher(self.robot, self.prof)

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()

    def close(self) -> None:
        if self.robot is not None:
            self.robot.disconnect()  # torque stays on: the arm holds its last pose
            self.robot = None

    # ---- information ---------------------------------------------------------------------
    def status(self, scan: bool = True) -> dict:
        q = read_q(self.robot)
        t = fk(q)
        err, load = self.sk.contact_signal()
        out = {
            "q_deg": fmt(q),
            "gripper_origin_mm": np.round(t[:3, 3] * 1000, 1).tolist(),
            "pen_tip_mm": np.round(self.sk.pen.tip(q) * 1000, 1).tolist(),
            "pen_pitch_deg": round(math.degrees(self.sk.pen.pitch(q)), 1),
            "servo_lag_ticks": err,
            "servo_load": load,
            "setup": self.prof.name,
        }
        if scan:
            out["unreachable_cells"] = self.sk.scan()
        return out

    # ---- motion ----------------------------------------------------------------------------
    def home(self, height_mm: float = 30.0) -> None:
        self.sk.home(height_mm / 1000.0)

    def park(self) -> None:
        self.sk.park()

    def jog(self, dx_mm=0.0, dy_mm=0.0, dz_mm=0.0) -> list[float]:
        target = self.sk.tip() + np.array([dx_mm, dy_mm, dz_mm]) / 1000.0
        self.sk.goto_xyz(target[:2], target[2], speed=0.03)
        return np.round(self.sk.tip() * 1000, 1).tolist()

    def snapshot(self, tag: str = "now", include_depth: bool = True) -> dict:
        return {k: str(v) if v else None for k, v in snapshot_all(tag, include_depth).items()}

    # ---- drawing ---------------------------------------------------------------------------
    def draw(self, design: str, only: list[int] | None = None, tag: str | None = None, **trace_kw) -> dict:
        """home -> draw -> park -> snapshots. ``only`` selects stroke indices (for touch-ups)."""
        strokes = get_design(design, tuple(self.prof.area_mm), **trace_kw)
        if only:
            strokes = [strokes[i] for i in only]
        tag = tag or Path(design).stem
        for cam in ("wrist", "c922"):
            snapshot(cam, f"{tag}-before")
        self.sk.home()
        t0 = time.time()
        stats = self.sk.draw(strokes)
        self.sk.park()
        time.sleep(0.5)
        shots = self.snapshot(f"{tag}-after")
        return {"design": design, "minutes": round((time.time() - t0) / 60, 2), **stats.as_dict(), "snapshots": shots}

    # ---- paper calibration -----------------------------------------------------------------
    def ladder(self, u0: float = 1.0, vs=(0.70, 0.40, 0.05), levels_mm=(12, 10.5, 9, 7.5, 6, 4.5)) -> dict:
        """Draw ink ladders (see Sketcher.ladder) and snapshot them for reading."""
        self.sk.home()
        marks = self.sk.ladder(u0, tuple(vs), tuple(levels_mm))
        WORK.mkdir(exist_ok=True)
        (WORK / "ladder.json").write_text(json.dumps(marks, indent=1))
        self.sk.park()
        time.sleep(0.5)
        return {"marks": marks, "snapshots": self.snapshot("ladder", include_depth=False)}

    def set_surface(self, z0_mm: float, slope_x_mm_per_m: float = 0.0, slope_y_mm_per_m: float = 0.0, note: str = "") -> dict:
        return set_surface(self.setup_path or DEFAULT_SETUP, z0_mm, slope_x_mm_per_m, slope_y_mm_per_m, note, self.prof)

    def probe(self, n: int = 3, save: bool = True) -> dict:
        """Servo-lag contact probing (stiff pen mounts). Fits and optionally saves the surface."""
        self.sk.home()
        surf, resid = self.sk.probe_grid(n)
        self.prof.surface = surf
        self.prof.surface_method = "servo_lag"
        if save and self.setup_path:
            self.prof.save(self.setup_path)
        return {"coef": surf.coef, "points": len(surf.points), "resid_mm": np.round(resid * 1000, 2).tolist()}


def set_surface(setup_path, z0_mm: float, slope_x_mm_per_m: float = 0.0, slope_y_mm_per_m: float = 0.0,
                note: str = "", prof: SetupProfile | None = None) -> dict:
    """Write an ink-ladder reading into a profile: paper at ``z0_mm`` at the drawing centre plus slopes."""
    prof = prof or SetupProfile.load(setup_path)
    c = prof.centre
    prof.surface = PaperSurface.from_ladder(float(c[0]), float(c[1]), z0_mm, slope_x_mm_per_m, slope_y_mm_per_m,
                                            note or f"ink ladder {time.strftime('%Y-%m-%d')}")
    prof.surface_method = "ink_ladder"
    path = prof.save(setup_path)
    return {"setup": str(path), "surface": prof.surface.coef}


def list_setups() -> list[str]:
    return sorted(str(p) for p in SETUPS.glob("*.json"))
