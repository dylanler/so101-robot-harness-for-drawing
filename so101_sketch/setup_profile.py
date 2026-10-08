"""A *setup profile* is everything that describes one physical drawing setup.

Pen mount (reference pose, length, lean), where the drawing sits on the table, the measured
paper height map, and the pen pressure / speed that worked. Profiles are JSON files in
``calibration/setups/``; as long as the arm, pen mount and sheet position are unchanged, any
design can be drawn again with the same profile and no re-calibration.

Coordinates are robot-base metres (x forward, y left, z up). The drawing frame has
``u`` = the viewer's right (viewer stands opposite the robot) and ``v`` = towards the robot,
derived from the radial direction of the drawing centre.
"""

from __future__ import annotations

import json
import math
import time
from dataclasses import asdict, dataclass, field
from pathlib import Path

import numpy as np


@dataclass
class PaperSurface:
    """Commanded tip z (m) at which the pen just touches the paper, as a polynomial in (x, y).

    This is *not* the true table height: it absorbs servo sag, calibration offsets and model
    error, so it is only valid for the pen mount and posture it was measured with.
    coef = [c0, cx, cy] (plane) or [c0, cx, cy, cxx, cxy, cyy] (quadratic) around (x0, y0).
    """

    x0: float
    y0: float
    coef: list[float]
    source: str = ""
    points: list[list[float]] = field(default_factory=list)  # [x, y, z] samples behind the fit

    def z(self, xy) -> float:
        c = np.asarray(self.coef, dtype=float)
        x, y = float(xy[0]) - self.x0, float(xy[1]) - self.y0
        basis = np.array([1.0, x, y, x * x, x * y, y * y][: len(c)])
        return float(c @ basis)

    @classmethod
    def fit(cls, pts, x0: float, y0: float, source: str) -> tuple["PaperSurface", np.ndarray]:
        """Least-squares plane (< 7 points) or quadratic (>= 7 points). Returns (surface, residuals_m)."""
        pts = np.asarray(pts, dtype=float)
        x, y = pts[:, 0] - x0, pts[:, 1] - y0
        if len(pts) >= 7:
            basis = np.stack([np.ones_like(x), x, y, x * x, x * y, y * y], axis=1)
        else:
            basis = np.stack([np.ones_like(x), x, y], axis=1)
        coef, *_ = np.linalg.lstsq(basis, pts[:, 2], rcond=None)
        return cls(x0, y0, coef.tolist(), source, pts.tolist()), pts[:, 2] - basis @ coef

    @classmethod
    def from_ladder(cls, x0: float, y0: float, z0_mm: float, slope_x_mm_per_m: float = 0.0,
                    slope_y_mm_per_m: float = 0.0, source: str = "ink ladder") -> "PaperSurface":
        """Plane read off an ink ladder: z0 at (x0, y0) plus slopes (mm of z per metre of x / y)."""
        return cls(x0, y0, [z0_mm / 1000.0, slope_x_mm_per_m / 1000.0, slope_y_mm_per_m / 1000.0], source)


@dataclass
class SetupProfile:
    name: str
    description: str
    # pen ---------------------------------------------------------------------------------
    pen_reference_q: dict[str, float]  # a pose with the pen pointing at the paper (picks the pen axis)
    pen_length_mm: float  # tip distance from the gripper frame origin along that axis
    pen_pitch_deg: float  # pen lean from vertical in the arm plane that the IK tries to keep
    home_seed: dict[str, float]  # joint posture that puts the IK on the right branch
    # where the drawing goes --------------------------------------------------------------
    centre_xy_m: list[float]
    area_mm: list[float]  # [width, height] of the drawing rectangle
    # how to draw -------------------------------------------------------------------------
    press_mm: float = 3.0  # command this far below the paper surface while drawing
    hover_mm: float = 8.0  # travel height above the surface between strokes
    speed_mm_s: float = 25.0
    step_mm: float = 1.0
    surface: PaperSurface | None = None
    # contact detection: "servo_lag" works for a stiff pen mount; "ink_ladder" for a flexible refill
    surface_method: str = "ink_ladder"
    notes: list[str] = field(default_factory=list)
    updated: float = 0.0

    # ---- geometry ------------------------------------------------------------------------
    @property
    def centre(self) -> np.ndarray:
        return np.asarray(self.centre_xy_m, dtype=float)

    @property
    def u_dir(self) -> np.ndarray:
        f = self.centre / np.linalg.norm(self.centre)
        return np.array([-f[1], f[0]])

    @property
    def v_dir(self) -> np.ndarray:
        return -self.centre / np.linalg.norm(self.centre)

    @property
    def area_m(self) -> tuple[float, float]:
        return self.area_mm[0] / 1000.0, self.area_mm[1] / 1000.0

    def paper_xy(self, u: float, v: float, w: float | None = None, h: float | None = None) -> np.ndarray:
        """Normalised drawing coordinates (u right, v up as seen in the reference image) -> base xy."""
        w0, h0 = self.area_m
        w = w0 if w is None else w
        h = h0 if h is None else h
        return self.centre + (u - 0.5) * w * self.u_dir + (v - 0.5) * h * self.v_dir

    def surface_z(self, xy) -> float:
        if self.surface is None:
            raise RuntimeError(f"setup '{self.name}' has no paper surface yet: run `ladder` or `probe` first")
        return self.surface.z(xy)

    @property
    def pitch_rad(self) -> float:
        return math.radians(self.pen_pitch_deg)

    # ---- io ------------------------------------------------------------------------------
    @classmethod
    def load(cls, path: str | Path) -> "SetupProfile":
        d = json.loads(Path(path).read_text())
        surf = d.pop("surface", None)
        prof = cls(**d)
        prof.surface = PaperSurface(**surf) if surf else None
        return prof

    def save(self, path: str | Path) -> Path:
        self.updated = time.time()
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(asdict(self), indent=2) + "\n")
        return path
