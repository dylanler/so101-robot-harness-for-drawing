"""Turn a photo or line drawing into pen strokes automatically.

Two modes:

* ``edges``   - Canny edges traced into polylines. Good for photos of buildings/objects.
* ``outline`` - contours of a dark-on-light threshold. Good for logos, silhouettes, line art.

The result is fitted (aspect preserved) into the unit square of the drawing rectangle, so the
same strokes work for any area in the setup profile. Hand-authored designs (petronas.py) give
cleaner robot drawings; tracing is the fast path for a new picture, and its output is a good
starting point to hand-edit (strokes are plain JSON).
"""

from __future__ import annotations

from pathlib import Path

import numpy as np

from .geometry import order_strokes

_NBRS = ((-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1))  # 4-neighbours first


def _trace_pixels(mask: np.ndarray) -> list[np.ndarray]:
    """Walk 1-px wide edge pixels into polylines, starting from line ends (degree-1 pixels)."""
    import cv2

    m = (mask > 0).astype(np.uint8)
    deg = cv2.filter2D(m, -1, np.ones((3, 3), np.float32), borderType=cv2.BORDER_CONSTANT) - m
    ys, xs = np.nonzero(m)
    remaining = set(zip(ys.tolist(), xs.tolist()))
    ends = [p for p in remaining if deg[p] == 1]
    starts = ends + [p for p in zip(ys.tolist(), xs.tolist())]
    paths = []
    for s in starts:
        if s not in remaining:
            continue
        remaining.discard(s)
        path, cur = [s], s
        while True:
            nxt = None
            for dy, dx in _NBRS:
                q = (cur[0] + dy, cur[1] + dx)
                if q in remaining:
                    nxt = q
                    break
            if nxt is None:
                break
            remaining.discard(nxt)
            path.append(nxt)
            cur = nxt
        paths.append(np.array([(x, y) for y, x in path], dtype=np.float32))
    return paths


def trace_image(
    path: str | Path,
    area_mm: tuple[float, float] = (130.0, 170.0),
    mode: str = "edges",
    max_px: int = 500,
    blur: int = 5,
    canny: tuple[int, int] | None = None,
    threshold: int | None = None,
    simplify_px: float = 1.5,
    min_len_px: float = 25.0,
    max_strokes: int = 250,
    margin: float = 0.03,
) -> list[np.ndarray]:
    """Return normalised strokes for ``path``. See module docstring for the modes."""
    import cv2

    img = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
    if img is None:
        raise FileNotFoundError(path)
    s = max_px / max(img.shape)
    if s < 1:
        img = cv2.resize(img, None, fx=s, fy=s, interpolation=cv2.INTER_AREA)
    if blur > 1:
        img = cv2.GaussianBlur(img, (blur | 1, blur | 1), 0)
    H, W = img.shape

    polys: list[np.ndarray] = []
    if mode == "edges":
        if canny is None:  # automatic thresholds around the median intensity
            med = float(np.median(img))
            canny = (int(max(0, 0.66 * med)), int(min(255, 1.33 * med)))
        edges = cv2.Canny(img, *canny)
        for p in _trace_pixels(edges):
            if len(p) >= 2:
                polys.append(cv2.approxPolyDP(p.reshape(-1, 1, 2), simplify_px, False).reshape(-1, 2))
    elif mode == "outline":
        thr = threshold if threshold is not None else 0
        flag = cv2.THRESH_BINARY_INV | (cv2.THRESH_OTSU if threshold is None else 0)
        _, bw = cv2.threshold(img, thr, 255, flag)
        contours, _ = cv2.findContours(bw, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
        for c in contours:
            a = cv2.approxPolyDP(c, simplify_px, True).reshape(-1, 2)
            if len(a) >= 2:
                polys.append(np.vstack([a, a[:1]]))
    else:
        raise ValueError(f"mode must be 'edges' or 'outline', got {mode!r}")

    def length(p):
        return float(np.sum(np.linalg.norm(np.diff(p, axis=0), axis=1))) if len(p) > 1 else 0.0

    polys = [p.astype(float) for p in polys if length(p) >= min_len_px]
    polys.sort(key=length, reverse=True)
    polys = polys[:max_strokes]

    # fit the image into the drawing rectangle (aspect preserved, centred, with a margin)
    aw, ah = area_mm
    scale = (1 - 2 * margin) * min(aw / W, ah / H)  # mm per px
    ox, oy = (aw - W * scale) / 2, (ah - H * scale) / 2
    strokes = [np.stack([(ox + p[:, 0] * scale) / aw, 1 - (oy + p[:, 1] * scale) / ah], axis=1) for p in polys]
    return order_strokes(strokes)
