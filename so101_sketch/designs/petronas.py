"""Stylised Petronas Twin Towers, Kuala Lumpur (references/petronas-twin-tower.jpeg)."""

from __future__ import annotations

import math

import numpy as np

from .geometry import bold

# half-widths (hw) at heights v: base -> setbacks -> crown
PROFILE = [(0.085, 0.12), (0.078, 0.55), (0.062, 0.55), (0.058, 0.68), (0.044, 0.68),
           (0.040, 0.77), (0.026, 0.77), (0.022, 0.83), (0.010, 0.83), (0.008, 0.87)]
TOWER_CENTRES = (0.36, 0.64)


def _shaft_hw(v: float) -> float:
    return 0.085 - (0.085 - 0.078) * (v - 0.12) / 0.43


def petronas_base() -> list[np.ndarray]:
    """Two tapered shafts with tiered setbacks and spires, skybridge + inverted-V legs, floor bands, podium, ground."""
    strokes: list = []
    for uc in TOWER_CENTRES:
        left = [(uc - hw, v) for hw, v in PROFILE]
        right = [(uc + hw, v) for hw, v in reversed(PROFILE)]
        # base -> up the left edge -> crown -> spire -> crown -> down the right edge -> base
        strokes.append(left + [(uc, 0.87), (uc, 0.97), (uc, 0.87)] + right)
        for v in (0.22, 0.32, 0.42, 0.62, 0.73):
            if v <= 0.55:
                hw = 0.082 - (0.082 - 0.078) * (v - 0.12) / 0.43
            else:
                hw = next(h for h, hv in reversed(PROFILE) if hv <= v)
            strokes.append([(uc - hw, v), (uc + hw, v)])
    # skybridge (double line) and its inverted-V legs meeting on the bridge
    strokes.append([(0.44, 0.485), (0.56, 0.485)])
    strokes.append([(0.44, 0.505), (0.56, 0.505), (0.56, 0.485)])
    strokes.append([(0.44, 0.36), (0.50, 0.485), (0.56, 0.36)])
    # podium + ground
    strokes.append([(0.20, 0.12), (0.20, 0.05), (0.80, 0.05), (0.80, 0.12)])
    strokes.append([(0.26, 0.085), (0.74, 0.085)])
    strokes.append([(0.02, 0.03), (0.98, 0.03)])
    return [np.asarray(s, dtype=float) for s in strokes]


def petronas_details() -> list[np.ndarray]:
    """Detail layer drawn on top of the base: facets, ribbon floors, bustle annexes, pinnacles, park, blocks, birds."""
    strokes: list = []
    for uc, side in ((TOWER_CENTRES[0], -1), (TOWER_CENTRES[1], +1)):
        # facet lines of the 8-point-star plan: two through the shaft, two through the first tier
        for f in (-0.032, 0.032):
            strokes.append([(uc + f * _shaft_hw(0.12) / 0.085, 0.12), (uc + f * _shaft_hw(0.55) / 0.085, 0.55)])
        for f in (-0.024, 0.024):
            strokes.append([(uc + f, 0.55), (uc + f, 0.68)])
        # stainless-steel ribbon floors between the base bands
        for v in (0.17, 0.27, 0.37, 0.47, 0.52):
            hw = _shaft_hw(v)
            strokes.append([(uc - hw, v), (uc + hw, v)])
        for v, hw in ((0.585, 0.062), (0.65, 0.062), (0.705, 0.044), (0.745, 0.044), (0.80, 0.026)):
            strokes.append([(uc - hw, v), (uc + hw, v)])
        # bustle annex on the outer side: stepped outline + mini spire + two ribbons
        o = side
        strokes.append([
            (uc + o * 0.085, 0.12), (uc + o * 0.135, 0.12), (uc + o * 0.135, 0.40), (uc + o * 0.124, 0.40),
            (uc + o * 0.124, 0.44), (uc + o * 0.112, 0.44), (uc + o * 0.112, 0.47), (uc + o * 0.085, 0.47),
        ])
        strokes.append([(uc + o * 0.108, 0.47), (uc + o * 0.108, 0.53)])
        for v in (0.22, 0.32):
            strokes.append([(uc + o * 0.135, v), (uc + o * 0.085, v)])
        # pinnacle ball (ellipse in uv so it is round on paper) + ring on the spire
        ang = np.linspace(0, 2 * np.pi, 13)
        strokes.append([(uc + 0.018 * math.cos(a), 0.885 + 0.014 * math.sin(a)) for a in ang])
        strokes.append([(uc - 0.012, 0.925), (uc + 0.012, 0.925)])
    # KLCC park: domed trees on the ground line either side of the podium
    for cu in (0.13, 0.175, 0.825, 0.87):
        ang = np.linspace(0, np.pi, 9)
        strokes.append([(cu + 0.022 * math.cos(a), 0.03 + 0.032 * math.sin(a)) for a in ang])
    # background blocks with window lines
    strokes.append([(0.03, 0.03), (0.03, 0.27), (0.10, 0.27), (0.10, 0.03)])
    strokes.append([(0.03, 0.11), (0.10, 0.11)])
    strokes.append([(0.03, 0.19), (0.10, 0.19)])
    strokes.append([(0.90, 0.03), (0.90, 0.21), (0.97, 0.21), (0.97, 0.03)])
    strokes.append([(0.90, 0.09), (0.97, 0.09)])
    strokes.append([(0.90, 0.15), (0.97, 0.15)])
    # birds
    strokes.append([(0.14, 0.86), (0.165, 0.84), (0.19, 0.86)])
    strokes.append([(0.80, 0.93), (0.825, 0.91), (0.85, 0.93)])
    return [np.asarray(s, dtype=float) for s in strokes]


def petronas_full() -> list[np.ndarray]:
    """The final drawing: base outlines twice (bolder silhouette), then the detail layer."""
    return bold(petronas_base()) + petronas_details()
