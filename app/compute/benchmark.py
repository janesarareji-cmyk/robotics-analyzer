"""R5.6: in-app benchmark. FK, workspace and batch IK throughput for a backend and the CPU.

``run_benchmark(backend, model, n)`` returns a dict (see ``BENCH_KEYS``). ``write_benchmark``
writes it as JSON with hardware and software versions. Batch IK always runs on the CPU
(R5.4 go/no-go: GPU IK is NO-GO), so its CPU and "backend" rows are the same CPU path.
"""
from __future__ import annotations

import datetime
import json
import os
import platform
import subprocess
import time
from pathlib import Path
from typing import Any

import numpy as np

import core_bridge as cb
from app.compute.backend import CpuBackend, estimate_reach
from app.kin_utils import dof_limits

BENCH_KEYS = ("n", "n_ik", "backend", "date", "fk", "workspace", "ik", "versions")
ROW_KEYS = ("samples_per_s", "ms", "info")
IK_N = 10_000


def _run(cmd) -> str:
    try:
        return subprocess.run(cmd, capture_output=True, text=True, timeout=15,
                              shell=isinstance(cmd, str)).stdout.strip()
    except Exception:
        return ""


def versions(backend: Any = None) -> dict:
    """Hardware and software versions. GPU fields are None when no device is used."""
    root = Path(__file__).resolve().parent.parent.parent
    v: dict[str, Any] = {
        "numpy": np.__version__, "python": platform.python_version(),
        "os": platform.platform(), "cpu": platform.processor() or "unknown",
        "cpu_threads": os.cpu_count() or 1,
        "git_commit": _run(["git", "-C", str(root), "rev-parse", "HEAD"]) or "unavailable",
        "gpu": None, "sm": None, "driver_version": None, "toolkit": None,
        "runtime_version": None,
    }
    try:
        import PySide6
        v["pyside6"] = PySide6.__version__
    except Exception:
        v["pyside6"] = "unavailable"
    if platform.system() == "Windows":
        out = _run('powershell -NoProfile -Command "(Get-CimInstance Win32_Processor).Name"')
        if out:
            v["cpu"] = out.splitlines()[0].strip()
    try:
        d = backend.device_info() if backend is not None else {}
    except Exception:
        d = {}
    if d.get("available"):
        v["gpu"] = d.get("name")
        v["sm"] = f"{d.get('sm_major')}.{d.get('sm_minor')}"
        v["driver_version"] = d.get("driver_version")
        v["runtime_version"] = d.get("runtime_version")
        v["vram_total_mb"] = int(d.get("total_bytes", 0)) // (1 << 20)
        nv = _run("nvcc --version").splitlines()
        v["toolkit"] = nv[-1].strip() if nv else "unavailable"
    return v


def _time(fn) -> tuple[Any, float]:
    t0 = time.perf_counter()
    r = fn()
    return r, (time.perf_counter() - t0) * 1e3


def _row(n: int, ms: float, info: dict) -> dict:
    return {"ms": ms, "samples_per_s": (n / (ms / 1000.0)) if ms > 0 else 0.0,
            "info": dict(info or {})}


def _ik_inputs(model, n: int, seed: int = 1):
    lo, hi = dof_limits(model)
    rng = np.random.default_rng(seed)
    dof = model.dof()
    qt = rng.uniform(lo, hi, size=(n, dof))
    fk = cb.fk_batch(cb.compile(model), qt)
    targets = np.zeros((n, 4, 4))
    targets[:, :3, :3] = fk["rotation"]
    targets[:, :3, 3] = fk["position"]
    targets[:, 3, 3] = 1.0
    q0 = np.clip(qt + rng.uniform(-0.5, 0.5, qt.shape) / np.sqrt(dof), lo, hi)
    return targets, q0


def _measure(be, model, n: int, n_ik: int, targets, q0, budget_bytes: int) -> dict:
    lo, hi = dof_limits(model)
    rng = np.random.default_rng(7)
    q = rng.uniform(lo, hi, size=(n, model.dof()))
    be.fk_batch(model, q[:min(n, 1000)])                       # warm-up
    fk_res, fk_ms = _time(lambda: be.fk_batch(model, q))
    voxel = max(estimate_reach(model) / 100.0, 1e-4)
    opts = dict(mode="random", n_samples=int(n), seed=42, voxel_size=voxel,
                want_counts=False, want_points=False, max_points=0)
    if budget_bytes > 0:
        opts["budget_bytes"] = int(budget_bytes)
    ws_res, ws_ms = _time(lambda: be.sample_workspace(model, opts))
    ik_res, ik_ms = _time(lambda: be.solve_ik_batch(model, targets[:n_ik], q0[:n_ik]))
    return {
        "fk": _row(n, fk_ms, fk_res.get("info")),
        "workspace": _row(n, ws_ms, ws_res.get("info")),
        "ik": _row(n_ik, ik_ms, ik_res.get("info")),
    }


def run_benchmark(backend: Any, model: Any, n: int = 1_000_000, n_ik: int = IK_N,
                  budget_bytes: int = 0) -> dict:
    """Benchmark `backend` and the CPU. Result keys: BENCH_KEYS.

    fk / workspace / ik each hold ``{"cpu": row, "backend": row or None, "speedup": x or None}``;
    a row has ``samples_per_s``, ``ms``, ``info``. ``backend`` is None when `backend` is the CPU.
    """
    if model is None or model.dof() <= 0:
        raise ValueError("benchmark needs a robot with at least one DOF")
    n = max(int(n), 1)
    n_ik = max(1, min(int(n_ik), n))
    targets, q0 = _ik_inputs(model, n_ik)
    cpu = CpuBackend("benchmark reference")
    cpu_rows = _measure(cpu, model, n, n_ik, targets, q0, budget_bytes)
    same = getattr(backend, "name", "cpu") == "cpu"
    be_rows = None if same else _measure(backend, model, n, n_ik, targets, q0, budget_bytes)
    out: dict[str, Any] = {"n": n, "n_ik": n_ik, "backend": getattr(backend, "name", "cpu"),
                           "date": datetime.datetime.now().isoformat(timespec="seconds")}
    for k in ("fk", "workspace", "ik"):
        row: dict[str, Any] = {"cpu": cpu_rows[k],
                               "backend": None if be_rows is None else be_rows[k],
                               "speedup": None}
        if be_rows is not None and be_rows[k]["ms"] > 0:
            row["speedup"] = cpu_rows[k]["ms"] / be_rows[k]["ms"]
        out[k] = row
    out["versions"] = versions(backend)
    return out


def write_benchmark(result: dict, path) -> Path:
    p = Path(path)
    p.parent.mkdir(parents=True, exist_ok=True)
    p.write_text(json.dumps(result, indent=2, default=str))
    return p
