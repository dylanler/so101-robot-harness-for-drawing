"""Design registry: built-in hand-authored designs, stroke JSON files and images."""

from __future__ import annotations

from pathlib import Path
from typing import Callable

import numpy as np

from .artscience import artscience_museum
from .geometry import bold, load_strokes, order_strokes, render_preview, save_strokes, stroke_stats
from .image_trace import trace_image
from .petronas import petronas_base, petronas_details, petronas_full

DESIGNS: dict[str, Callable[[], list[np.ndarray]]] = {
    "petronas": petronas_full,
    "petronas-base": petronas_base,
    "petronas-details": petronas_details,
    "artscience": artscience_museum,
}

IMAGE_SUFFIXES = {".jpg", ".jpeg", ".png", ".bmp", ".webp"}


def get_design(spec: str, area_mm: tuple[float, float] = (130.0, 170.0), **trace_kw) -> list[np.ndarray]:
    """``spec`` is a built-in name, a ``.json`` stroke file, or an image (traced on the fly)."""
    if spec in DESIGNS:
        return DESIGNS[spec]()
    p = Path(spec)
    if p.suffix.lower() == ".json":
        return load_strokes(p)
    if p.suffix.lower() in IMAGE_SUFFIXES:
        return trace_image(p, area_mm=area_mm, **trace_kw)
    raise KeyError(f"unknown design {spec!r}; built-ins: {sorted(DESIGNS)} (or pass a .json / image path)")


__all__ = [
    "DESIGNS", "get_design", "trace_image", "render_preview", "save_strokes", "load_strokes",
    "stroke_stats", "order_strokes", "bold",
]
