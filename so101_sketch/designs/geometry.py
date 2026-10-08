"""Stroke helpers shared by designs: resampling, curves, ordering, preview rendering, JSON io.

A *design* is a list of strokes; a stroke is an (N, 2) array of normalised points (u, v) in
[0, 1]^2 with u to the right and v up, as you would see the reference image. The drawing
rectangle size and placement come from the setup profile, so designs are size-independent.
"""

from __future__ import annotations

import json
import math
from pathlib import Path

import numpy as np


def resample(pts, step: float) -> np.ndarray:
    """Points every ``step`` along a polyline (same units as pts)."""
    pts = np.asarray(pts, dtype=np.float64).reshape(-1, 2)
    if len(pts) < 2:
        return pts
    d = np.sqrt((np.diff(pts, axis=0) ** 2).sum(axis=1))
    s = np.concatenate([[0.0], np.cumsum(d)])
    if s[-1] < 1e-9:
        return pts[:1]
    n = max(2, int(math.ceil(s[-1] / step)) + 1)
    t = np.linspace(0.0, s[-1], n)
    return np.stack([np.interp(t, s, pts[:, 0]), np.interp(t, s, pts[:, 1])], axis=1)


def bezier(p0, c, p1, n: int = 16) -> np.ndarray:
    """Quadratic Bezier from p0 to p1 with control point c."""
    p0, c, p1 = (np.asarray(v, dtype=float) for v in (p0, c, p1))
    t = np.linspace(0, 1, n)[:, None]
    return (1 - t) ** 2 * p0 + 2 * (1 - t) * t * c + t**2 * p1


def ellipse(cu: float, cv: float, ru: float, rv: float, a0: float = 0.0, a1: float = 2 * math.pi, n: int = 13) -> np.ndarray:
    ang = np.linspace(a0, a1, n)
    return np.stack([cu + ru * np.cos(ang), cv + rv * np.sin(ang)], axis=1)


def bold(strokes: list[np.ndarray]) -> list[np.ndarray]:
    """Each stroke followed by its reverse: a second pass darkens a light pen without offsetting it."""
    return list(strokes) + [s[::-1].copy() for s in strokes]


def order_strokes(strokes: list[np.ndarray]) -> list[np.ndarray]:
    """Greedy nearest-neighbour ordering (may reverse strokes) to cut pen-up travel."""
    rest = [np.asarray(s, dtype=float) for s in strokes if len(s)]
    out: list[np.ndarray] = []
    cur = np.array([0.5, 0.5])
    while rest:
        best, best_d, flip = 0, math.inf, False
        for i, s in enumerate(rest):
            for f, end in ((False, s[0]), (True, s[-1])):
                d = float(np.sum((end - cur) ** 2))
                if d < best_d:
                    best, best_d, flip = i, d, f
        s = rest.pop(best)
        s = s[::-1] if flip else s
        out.append(s)
        cur = s[-1]
    return out


def stroke_stats(strokes: list[np.ndarray], area_mm: tuple[float, float] = (1.0, 1.0)) -> dict:
    w, h = area_mm
    ink = sum(float(np.sum(np.hypot(np.diff(s[:, 0]) * w, np.diff(s[:, 1]) * h))) for s in strokes if len(s) > 1)
    travel, cur = 0.0, None
    for s in strokes:
        if cur is not None:
            travel += float(math.hypot((s[0, 0] - cur[0]) * w, (s[0, 1] - cur[1]) * h))
        cur = s[-1]
    return {"strokes": len(strokes), "points": int(sum(len(s) for s in strokes)), "ink": round(ink, 1), "travel": round(travel, 1)}


def render_preview(strokes: list[np.ndarray], out: Path, area_mm=(130.0, 170.0), px_per_mm: float = 5.0) -> Path:
    """White PNG with the strokes in black, aspect matching the drawing rectangle."""
    import cv2

    W, H = int(area_mm[0] * px_per_mm), int(area_mm[1] * px_per_mm)
    img = np.full((H, W, 3), 255, np.uint8)
    for s in strokes:
        pts = np.stack([s[:, 0] * W, (1 - s[:, 1]) * H], axis=1).astype(np.int32)
        cv2.polylines(img, [pts], False, (0, 0, 0), 2, lineType=cv2.LINE_AA)
    out.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(out), img)
    return out


def save_strokes(strokes: list[np.ndarray], path: Path, meta: dict | None = None) -> Path:
    data = {"format": "so101-strokes/v1", "meta": meta or {}, "strokes": [np.round(s, 5).tolist() for s in strokes]}
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data) + "\n")
    return path


def load_strokes(path: Path) -> list[np.ndarray]:
    data = json.loads(Path(path).read_text())
    strokes = data["strokes"] if isinstance(data, dict) else data
    return [np.asarray(s, dtype=float) for s in strokes if len(s) >= 2]
