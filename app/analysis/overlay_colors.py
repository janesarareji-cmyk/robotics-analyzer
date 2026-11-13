"""U6.2: colour helpers of the overlays (one place, no matplotlib import at module load)."""
from __future__ import annotations

import numpy as np

RED = (255, 48, 48)          # singular (#ff3030)
ORANGE = (255, 152, 0)       # warning
GREEN = (102, 187, 106)
_LUT: dict[str, np.ndarray] = {}


def _lut(name: str) -> np.ndarray:
    if name not in _LUT:
        try:
            from matplotlib import colormaps
            cm = colormaps[name]
            _LUT[name] = (np.asarray([cm(i / 255.0)[:3] for i in range(256)]) * 255.0).round().astype(np.uint8)
        except Exception:   # matplotlib missing: blue -> yellow ramp
            t = np.linspace(0, 1, 256)[:, None]
            _LUT[name] = (np.array([[40, 60, 200]]) * (1 - t) + np.array([250, 230, 40]) * t).astype(np.uint8)
    return _LUT[name]


def colormap_rgb(name: str, t) -> np.ndarray:
    """(N, 3) uint8 colours of ``t`` in 0..1 (clipped)."""
    lut = _lut(name)
    idx = np.clip(np.asarray(t, dtype=float) * 255.0, 0, 255).astype(np.int64)
    return lut[idx]


def colormap_stops(name: str, n: int = 6) -> list:
    """Legend stops [(pos, (r, g, b))] of a colour map."""
    lut = _lut(name)
    return [(i / (n - 1), tuple(int(v) for v in lut[int(round(i / (n - 1) * 255))])) for i in range(n)]
