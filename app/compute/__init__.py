"""R5.5: compute backends (CPU reference, optional CUDA) and VRAM budget helpers."""
from app.compute.backend import (  # noqa: F401
    Backend, CpuBackend, GpuBackend, get_backend, KEY_BACKEND, ENV_FORCE_CPU,
)
from app.compute.budget import (  # noqa: F401
    KEY_VRAM_BUDGET_MB, vram_budget_bytes, max_batch,
)
