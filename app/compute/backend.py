"""R5.5: one Python compute API over the CPU reference and the optional CUDA module.

``get_backend(preference)`` returns a ``CpuBackend`` or ``GpuBackend``. Both have the same
methods and the same result dicts (``info`` always has ``backend``, ``fallback``,
``fallback_reason``). The GPU backend re-runs a call on the CPU backend when the GPU cannot
finish it (out of memory below the minimum chunk, sticky CUDA error) and logs a warning.
Batch IK always runs on the CPU (R5.4 go/no-go: GPU IK is NO-GO) unless ``USE_GPU_IK``
is set (experimental, off).
"""
from __future__ import annotations

import os
from typing import Any

import numpy as np

import core_bridge as cb
from app.kin_utils import dof_limits
from app.logger import log

KEY_BACKEND = "compute/backend"
ENV_FORCE_CPU = "ANALYZER_FORCE_CPU"
PREFERENCES = ("auto", "cpu", "cuda")
USE_GPU_IK = False  # R5.4: GPU IK is NO-GO; gpu_bridge.solve_ik_batch is experimental

_NOT_A_FALLBACK = ("", "empty input")


def _import_gpu_bridge():
    """Import the optional CUDA module; None when it cannot be imported."""
    try:
        import gpu_bridge
        return gpu_bridge
    except Exception:
        return None


def _gpu_bridge_or_reason() -> tuple[Any, str]:
    """(module, "") when the GPU is usable, else (None, reason)."""
    gb = _import_gpu_bridge()
    if gb is None:
        return None, "gpu_bridge not importable"
    try:
        di = gb.device_info()
    except Exception as e:
        return None, f"gpu_bridge device query failed: {e}"
    if not di.get("available"):
        return None, f"no device ({di.get('error') or 'unavailable'})"
    try:
        if gb.is_dead():
            return None, "GPU marked dead by a sticky CUDA error"
    except Exception:
        pass
    return gb, ""


def estimate_reach(model_or_arrays: Any, limits: tuple | None = None) -> float:
    """Upper bound of the TCP distance: sum of all translation norms (pre, post, tool)
    plus ``max(|lo|, |hi|)`` stroke of every prismatic joint (same rule as C++ ``reach_bound``).

    ``limits`` = ``(lo, hi)`` per DOF; taken from the model when a ``RobotModel`` is given.
    An arrays dict without ``limits`` has no stroke information: prismatic strokes are not added.
    """
    is_arrays = isinstance(model_or_arrays, dict)
    a = model_or_arrays if is_arrays else cb.compile_arrays(model_or_arrays)
    if limits is None and not is_arrays:
        limits = dof_limits(model_or_arrays)
    tot = 0.0
    for key in ("pre", "post"):
        m = np.asarray(a[key], dtype=float).reshape(-1, 16)
        tot += float(np.linalg.norm(m[:, [3, 7, 11]], axis=1).sum())
    tool = np.asarray(a["tool"], dtype=float).reshape(16)
    tot += float(np.linalg.norm(tool[[3, 7, 11]]))
    if limits is not None:
        lo, hi = np.asarray(limits[0], float), np.asarray(limits[1], float)
        types = np.asarray(a["type"]).reshape(-1)
        src = np.asarray(a["src_q_index"]).reshape(-1)
        mult = np.asarray(a["mult"], float).reshape(-1)
        off = np.asarray(a["offset"], float).reshape(-1)
        axis = np.asarray(a["axis"], float).reshape(-1, 3)
        for k in range(len(types)):
            if int(types[k]) != int(cb.JointType.Prismatic) or not (0 <= int(src[k]) < len(lo)):
                continue
            s = int(src[k])
            ends = (mult[k] * lo[s] + off[k], mult[k] * hi[s] + off[k])
            tot += max(abs(ends[0]), abs(ends[1])) * float(np.linalg.norm(axis[k]))
    return tot


class Backend:
    """Common behaviour. Subclasses: CpuBackend, GpuBackend."""

    name = "base"

    def __init__(self, reason: str = ""):
        self.reason = reason

    @staticmethod
    def _need_model(model: Any):
        if isinstance(model, dict):
            raise ValueError("this call needs a RobotModel, not an arrays dict")
        return model

    @staticmethod
    def _finish_info(res: dict, backend: str, fallback: bool = False, reason: str = "") -> dict:
        info = dict(res.get("info") or {})
        info["backend"] = info.get("backend", backend)
        info["fallback"] = bool(fallback)
        info["fallback_reason"] = reason if fallback else ""
        res["info"] = info
        return res

    def device_info(self) -> dict:  # pragma: no cover - overridden
        raise NotImplementedError


class CpuBackend(Backend):
    name = "cpu"

    def device_info(self) -> dict:
        return {"available": False, "name": "CPU", "threads": os.cpu_count() or 1,
                "backend": "cpu", "reason": self.reason}

    def fk_batch(self, model_or_arrays, q, precision: str = "fp32", frame: str = "world") -> dict:
        if precision not in ("fp32", "fp64"):
            raise ValueError("precision must be 'fp32' or 'fp64'")
        model = self._need_model(model_or_arrays)
        res = cb.fk_batch(cb.compile(model), np.asarray(q, dtype=float), frame)
        res["info"] = {"backend": "cpu", "precision": "fp64"}   # CPU always computes in double
        return self._finish_info(res, "cpu")

    def sample_workspace(self, model, options: dict, cancel=None, progress=None) -> dict:
        model = self._need_model(model)
        lo, hi = dof_limits(model)
        res = cb.sample_workspace(cb.compile(model), lo, hi, dict(options), cancel, progress)
        return self._finish_info(res, "cpu")

    def solve_ik_batch(self, model, targets, q0, options: dict | None = None,
                       n_threads: int = 0) -> dict:
        model = self._need_model(model)
        lo, hi = dof_limits(model)
        res = cb.solve_ik_batch(cb.compile(model), lo, hi, np.asarray(targets, dtype=float),
                                np.asarray(q0, dtype=float), options, n_threads)
        res["info"] = {"backend": "cpu"}
        return self._finish_info(res, "cpu")


class GpuBackend(Backend):
    name = "cuda"

    def __init__(self, gb: Any, reason: str = "", cpu: CpuBackend | None = None):
        super().__init__(reason)
        self._gb = gb
        self._cpu = cpu or CpuBackend()

    def device_info(self) -> dict:
        d = dict(self._gb.device_info())
        d["backend"] = "cuda"
        d["reason"] = self.reason
        return d

    def mem_info(self) -> dict:
        return dict(self._gb.mem_info())

    def _run(self, what: str, gpu_call, cpu_call) -> dict:
        """Run `gpu_call`; on a CUDA failure re-run `cpu_call`. ValueError propagates."""
        reason = ""
        try:
            if self._gb.is_dead():
                reason = "GPU marked dead by a sticky CUDA error"
        except Exception:
            pass
        if not reason:
            try:
                res = gpu_call()
                info = res.get("info") or {}
                r = str(info.get("fallback_reason") or "")
                if info.get("backend") == "cpu" and r not in _NOT_A_FALLBACK:
                    log.warning("%s: GPU could not run, used the CPU (%s)", what, r)
                    return self._finish_info(res, "cpu", True, r)
                return self._finish_info(res, "cuda")
            except ValueError:
                raise
            except Exception as e:   # RuntimeError from CUDA_CHECK, sticky error, ...
                reason = f"CUDA error: {e}"
        log.warning("%s: GPU failed, re-running on the CPU (%s)", what, reason)
        res = cpu_call()
        return self._finish_info(res, "cpu", True, reason)

    def fk_batch(self, model_or_arrays, q, precision: str = "fp32", frame: str = "world") -> dict:
        arrays = (model_or_arrays if isinstance(model_or_arrays, dict)
                  else cb.compile_arrays(model_or_arrays))
        qa = np.asarray(q, dtype=float)
        return self._run(
            "fk_batch",
            lambda: self._gb.fk_batch(arrays, qa, precision, frame),
            lambda: self._cpu.fk_batch(self._need_model(model_or_arrays), qa, precision, frame))

    def sample_workspace(self, model, options: dict, cancel=None, progress=None) -> dict:
        model = self._need_model(model)
        lo, hi = dof_limits(model)
        arrays = cb.compile_arrays(model)
        opts = dict(options)
        return self._run(
            "sample_workspace",
            lambda: self._gb.sample_workspace(arrays, lo, hi, opts, cancel, progress),
            lambda: self._cpu.sample_workspace(model, opts, cancel, progress))

    def solve_ik_batch(self, model, targets, q0, options: dict | None = None,
                       n_threads: int = 0) -> dict:
        if not USE_GPU_IK:   # R5.4 decision: CPU batch IK
            return self._cpu.solve_ik_batch(model, targets, q0, options, n_threads)
        model = self._need_model(model)
        lo, hi = dof_limits(model)
        arrays = cb.compile_arrays(model)
        t = np.asarray(targets, dtype=float)
        q = np.asarray(q0, dtype=float)
        return self._run(
            "solve_ik_batch",
            lambda: self._gb.solve_ik_batch(arrays, lo, hi, t, q, options, 0, n_threads),
            lambda: self._cpu.solve_ik_batch(model, t, q, options, n_threads))


def get_backend(preference: str | None = None, settings: Any = None) -> Backend:
    """Pick a backend. ``backend.reason`` says why.

    Order: env ``ANALYZER_FORCE_CPU=1`` (always wins) > argument > setting
    ``compute/backend`` > ``"auto"``.
    """
    if os.environ.get(ENV_FORCE_CPU, "").strip() not in ("", "0"):
        return CpuBackend(f"forced by env {ENV_FORCE_CPU}")
    if preference is None:
        if settings is None:
            from app.settings import AppSettings
            settings = AppSettings()
        preference = str(settings.get(KEY_BACKEND, "auto"))
    if preference not in PREFERENCES:
        log.warning("unknown compute backend '%s', using auto", preference)
        preference = "auto"
    if preference == "cpu":
        return CpuBackend("cpu selected")
    gb, why = _gpu_bridge_or_reason()
    if gb is None:
        if preference == "cuda":
            log.warning("cuda requested but unavailable (%s), using the CPU", why)
            return CpuBackend(f"cuda requested but unavailable: {why}")
        return CpuBackend(why)
    return GpuBackend(gb, "cuda selected" if preference == "cuda" else "auto: GPU available")
