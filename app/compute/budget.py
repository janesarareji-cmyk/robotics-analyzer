"""R5.5: VRAM budget from settings and the live chunk-size rule."""
from __future__ import annotations

from typing import Any

import core_bridge as cb

KEY_VRAM_BUDGET_MB = "compute/vram_budget_mb"
DEFAULT_VRAM_BUDGET_MB = 6144
MIN_VRAM_BUDGET_MB = 256
MAX_VRAM_BUDGET_MB = 7168
CPU_MAX_BATCH = 1_000_000


def vram_budget_mb(settings: Any = None) -> int:
    """Budget in MiB from the settings (clamped to 256..7168, default 6144)."""
    if settings is None:
        from app.settings import AppSettings
        settings = AppSettings()
    try:
        v = int(float(settings.get(KEY_VRAM_BUDGET_MB, DEFAULT_VRAM_BUDGET_MB)))
    except (TypeError, ValueError):
        v = DEFAULT_VRAM_BUDGET_MB
    return max(MIN_VRAM_BUDGET_MB, min(MAX_VRAM_BUDGET_MB, v))


def vram_budget_bytes(settings: Any = None) -> int:
    return vram_budget_mb(settings) * 1024 * 1024


def set_vram_budget_mb(settings: Any, mb: int) -> int:
    """Store the budget (clamped). Returns the stored value."""
    mb = max(MIN_VRAM_BUDGET_MB, min(MAX_VRAM_BUDGET_MB, int(mb)))
    settings.set(KEY_VRAM_BUDGET_MB, mb)
    return mb


def _gpu_mem_info() -> dict | None:
    from app.compute import backend as _b
    gb = _b._import_gpu_bridge()
    if gb is None:
        return None
    try:
        mi = gb.mem_info()
    except Exception:
        return None
    return mi if mi.get("ok") else None


def max_batch(bytes_per_item: int, settings: Any = None, persistent_bytes: int = 0) -> int:
    """Largest batch (items) that fits the budget and the live free VRAM.

    CPU backend (no usable GPU): fixed 1e6. 0 from the rule means "use the CPU".
    """
    mi = _gpu_mem_info()
    if mi is None:
        return CPU_MAX_BATCH
    return int(cb.chunk_size(int(mi["free_bytes"]), vram_budget_bytes(settings),
                             int(persistent_bytes), int(bytes_per_item)))
