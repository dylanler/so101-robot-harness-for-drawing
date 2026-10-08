"""Stylised ArtScience Museum, Singapore (references/artscience-museum.jpeg)."""

from __future__ import annotations

import math

import numpy as np

from .geometry import bezier


def artscience_museum() -> list[np.ndarray]:
    """Lotus bowl with six petals, skylights, struts, platform/railing/sea wall and water."""
    strokes: list = []

    # bowl + outer silhouette (left tall petal outer edge -> bowl -> right petal outer edge)
    strokes.append(
        np.concatenate(
            [
                bezier((0.11, 0.86), (0.07, 0.62), (0.19, 0.49), 14),
                bezier((0.19, 0.49), (0.50, 0.30), (0.81, 0.49), 22),
                bezier((0.81, 0.49), (0.91, 0.55), (0.94, 0.63), 10),
            ]
        )
    )

    # petals: (base point on bowl, top-left, top-right). Edges bow outwards, away from u=0.5.
    petals = [
        ((0.22, 0.47), (0.11, 0.86), (0.25, 0.83)),
        ((0.33, 0.42), (0.27, 0.78), (0.40, 0.76)),
        ((0.44, 0.39), (0.42, 0.75), (0.53, 0.74)),
        ((0.55, 0.39), (0.55, 0.72), (0.65, 0.70)),
        ((0.66, 0.42), (0.67, 0.67), (0.78, 0.64)),
        ((0.77, 0.47), (0.80, 0.63), (0.94, 0.63)),
    ]
    for base, tl, tr in petals:
        bx = np.array(base)

        def bow(a, b):
            m = (np.asarray(a) + np.asarray(b)) / 2
            side = -1.0 if m[0] < 0.5 else 1.0
            return m + np.array([0.045 * side, 0.0])

        left = bezier(bx, bow(bx, tl), tl, 12)
        top = np.array([tl, tr])
        right = bezier(tr, bow(tr, bx + np.array([0.03, 0.0])), bx + np.array([0.03, 0.0]), 12)
        strokes.append(np.concatenate([left, top, right]))

    # skylight windows on the four central petals
    for cu, cv in ((0.335, 0.66), (0.475, 0.64), (0.60, 0.62), (0.725, 0.575)):
        w, h = 0.035, 0.022
        strokes.append([(cu - w, cv + h), (cu + w, cv + h), (cu + w, cv - h), (cu - w, cv - h), (cu - w, cv + h)])

    # struts under the bowl to the platform
    for u0, u1 in ((0.36, 0.33), (0.43, 0.42), (0.50, 0.50), (0.57, 0.58), (0.64, 0.67)):
        strokes.append([(u0, 0.375), (u1, 0.29)])
    # platform, railing, sea wall
    strokes.append([(0.16, 0.29), (0.84, 0.29)])
    strokes.append([(0.10, 0.26), (0.90, 0.26)])
    strokes.append([(0.06, 0.235), (0.94, 0.235)])
    # water
    for v0 in (0.185, 0.13, 0.075):
        us = np.linspace(0.04, 0.96, 40)
        strokes.append(np.stack([us, v0 + 0.012 * np.sin(us * 2 * math.pi * 3.0)], axis=1))
    return [np.asarray(s, dtype=float) for s in strokes]
