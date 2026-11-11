"""R5.2 benchmark: batch FK, GPU fp32 / GPU fp64 / CPU (all threads).

N = 1e6 samples, 12-joint origin_axis robot. Each variant: 2 warm-up calls, then
5 timed repeats, median reported. Writes benchmarks/r5_fk_<date>.json with
hardware and software versions and applies the R5.2 precision rule:

    default = fp64 ONLY IF e2e_fp64 <= 2 * e2e_fp32 AND e2e_fp64 <= 50 ms
    (medians at N = 1e6), else fp32.

Run from the repo root:  python tools/bench_fk.py [--n 1000000] [--repeats 5]
"""
import argparse
import datetime
import json
import os
import platform
import re
import statistics
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "tests" / "python"))

import numpy as np  # noqa: E402

import core_bridge as cb  # noqa: E402
import gpu_bridge as gb  # noqa: E402
from robots import random_robot  # noqa: E402


def _run(cmd):
    try:
        return subprocess.run(cmd, capture_output=True, text=True, timeout=30, shell=isinstance(cmd, str)).stdout.strip()
    except Exception as e:  # pragma: no cover
        return f"unavailable ({e})"


def _cpu_name():
    out = _run('powershell -NoProfile -Command "(Get-CimInstance Win32_Processor).Name"')
    return out.splitlines()[0].strip() if out else platform.processor()


def _msvc_version():
    for f in (ROOT / "build" / "CMakeFiles").glob("*/CMakeCXXCompiler.cmake"):
        m = re.search(r'set\(CMAKE_CXX_COMPILER_VERSION "([^"]+)"', f.read_text())
        if m:
            return m.group(1)
    return "unknown"


def _toolkit():
    out = _run("nvcc --version")
    m = re.search(r"release ([\d.]+), V([\d.]+)", out)
    return f"{m.group(1)} (V{m.group(2)})" if m else out


def _versions(dev):
    smi = _run("nvidia-smi --query-gpu=driver_version,name,memory.total --format=csv,noheader")
    return {
        "gpu_name": dev["name"],
        "gpu_sm": f"{dev['sm_major']}.{dev['sm_minor']}",
        "gpu_total_bytes": dev["total_bytes"],
        "nvidia_smi": smi,
        "driver_cuda_version": dev["driver_version"],
        "cuda_runtime_version": dev["runtime_version"],
        "cuda_toolkit": _toolkit(),
        "msvc": _msvc_version(),
        "numpy": np.__version__,
        "python": platform.python_version(),
        "os": platform.platform(),
        "cpu": _cpu_name(),
        "cpu_threads": os.cpu_count(),
        "git_commit": _run(["git", "-C", str(ROOT), "rev-parse", "HEAD"]),
    }


def _median(xs):
    return statistics.median(xs)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--n", type=int, default=1_000_000)
    ap.add_argument("--repeats", type=int, default=5)
    ap.add_argument("--out", default=None)
    args = ap.parse_args()

    dev = gb.device_info()
    if not dev["available"]:
        sys.exit(f"no CUDA device: {dev['error']}")

    model = random_robot(12, 12)  # 12R origin_axis
    cm = cb.compile(model)
    arr = cb.compile_arrays(model)
    lo = np.array([j.limits.lower for j in model.joints])
    hi = np.array([j.limits.upper for j in model.joints])
    q = np.random.default_rng(42).uniform(lo, hi, size=(args.n, len(lo)))

    results = {}
    for prec in ("fp32", "fp64"):
        for _ in range(2):
            gb.fk_batch(arr, q, precision=prec)
        e2e, kern, last = [], [], None
        for _ in range(args.repeats):
            t0 = time.perf_counter()
            r = gb.fk_batch(arr, q, precision=prec)
            e2e.append((time.perf_counter() - t0) * 1e3)
            kern.append(r["info"]["kernel_ms"])
            last = r["info"]
        results[f"gpu_{prec}"] = {
            "kernel_ms_median": _median(kern), "e2e_ms_median": _median(e2e),
            "kernel_ms_all": kern, "e2e_ms_all": e2e,
            "backend": last["backend"], "chunks": last["chunks"], "chunk": last["chunk"],
        }

    for _ in range(2):
        cb.fk_batch(cm, q)
    cpu = []
    for _ in range(args.repeats):
        t0 = time.perf_counter()
        cb.fk_batch(cm, q)
        cpu.append((time.perf_counter() - t0) * 1e3)
    results["cpu_all_threads"] = {"e2e_ms_median": _median(cpu), "e2e_ms_all": cpu}

    e32 = results["gpu_fp32"]["e2e_ms_median"]
    e64 = results["gpu_fp64"]["e2e_ms_median"]
    cond_ratio = e64 <= 2.0 * e32
    cond_abs = e64 <= 50.0
    default = "fp64" if (cond_ratio and cond_abs) else "fp32"
    decision = {
        "rule": "default fp64 only if e2e_fp64 <= 2 * e2e_fp32 AND e2e_fp64 <= 50 ms (N = 1e6, 12 joints)",
        "e2e_fp32_ms": e32, "e2e_fp64_ms": e64,
        "ratio_fp64_over_fp32": e64 / e32,
        "cond_ratio": cond_ratio, "cond_abs_50ms": cond_abs,
        "default_precision": default,
    }

    out = {
        "benchmark": "r5_fk",
        "date": datetime.date.today().isoformat(),
        "n_samples": args.n, "n_joints": 12, "repeats": args.repeats,
        "robot": "random_robot(seed=12, n=12), origin_axis revolute",
        "versions": _versions(dev),
        "results": results,
        "decision": decision,
    }
    path = Path(args.out) if args.out else ROOT / "benchmarks" / f"r5_fk_{out['date']}.json"
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(out, indent=2))
    print(json.dumps({"results": {k: {kk: vv for kk, vv in v.items() if kk.endswith("median")}
                                  for k, v in results.items()}, "decision": decision}, indent=2))
    print(f"written {path}")


if __name__ == "__main__":
    main()
