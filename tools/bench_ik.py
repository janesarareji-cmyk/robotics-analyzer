"""R5.4 benchmark: batch IK. CPU (all threads) and GPU fp32 + CPU fp64 polish (go/no-go).

Targets: random reachable on the 6-DOF fixture, start within 0.5 rad / sqrt(dof)
per joint of q_true. Median of --repeats timed runs after 1 warm-up.
Writes benchmarks/r5_ik_<date>.json.

Run from the repo root:  python tools/bench_ik.py [--n 100000] [--repeats 3]
"""
import argparse
import datetime
import json
import os
import platform
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
from app.kin_utils import dof_limits  # noqa: E402
from robots import fk_test_robots  # noqa: E402


def _run(cmd):
    try:
        return subprocess.run(cmd, capture_output=True, text=True, timeout=30,
                              shell=isinstance(cmd, str)).stdout.strip()
    except Exception as e:  # pragma: no cover
        return f"unavailable ({e})"


def _cpu_name():
    out = _run('powershell -NoProfile -Command "(Get-CimInstance Win32_Processor).Name"')
    return out.splitlines()[0].strip() if out else platform.processor()


def make_targets(n, seed=1):
    """Reachable targets via batch FK of random q (cb.fk_batch row-major 12 -> 4x4)."""
    model = fk_test_robots()["six_dof_fixture"]
    cm = cb.compile(model)
    lo, hi = dof_limits(model)
    rng = np.random.default_rng(seed)
    dof = model.dof()
    qt = rng.uniform(lo, hi, size=(n, dof))
    return model, cm, lo, hi, qt, rng


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--n", type=int, default=100000)
    ap.add_argument("--repeats", type=int, default=3)
    ap.add_argument("--out", default=None)
    args = ap.parse_args()

    model, cm, lo, hi, qt, rng = make_targets(args.n)
    dof = model.dof()
    fk = cb.fk_batch(cm, qt)  # double CPU batch FK (validated against numpy FK in R5.2)
    targets = np.zeros((args.n, 4, 4))
    targets[:, :3, :3] = fk["rotation"]
    targets[:, :3, 3] = fk["position"]
    targets[:, 3, 3] = 1.0
    q0 = np.clip(qt + rng.uniform(-0.5, 0.5, qt.shape) / np.sqrt(dof), lo, hi)

    cb.solve_ik_batch(cm, lo, hi, targets[:1000], q0[:1000])  # warm-up
    times, rate = [], None
    for _ in range(args.repeats):
        t0 = time.perf_counter()
        r = cb.solve_ik_batch(cm, lo, hi, targets, q0)
        times.append((time.perf_counter() - t0) * 1e3)
        rate = float((r["status"] == cb.IK_CONVERGED).mean())
    counts = {int(k): int((r["status"] == k).sum()) for k in range(4)}
    cpu_ms = statistics.median(times)

    gpu_res, gpu_ver = None, {}
    try:
        import gpu_bridge as gb
        dev = gb.device_info()
        if dev["available"]:
            gpu_ver = {"gpu": dev["name"], "sm": f"{dev['sm_major']}.{dev['sm_minor']}",
                       "driver_version": dev["driver_version"],
                       "runtime_version": dev["runtime_version"],
                       "vram_total_mb": dev["total_bytes"] // (1 << 20)}
            arrays = cb.compile_arrays(model)
            gb.solve_ik_batch(arrays, lo, hi, targets[:1000], q0[:1000])  # warm-up
            gt, infos = [], []
            for _ in range(args.repeats):
                t0 = time.perf_counter()
                g = gb.solve_ik_batch(arrays, lo, hi, targets, q0)
                gt.append((time.perf_counter() - t0) * 1e3)
                infos.append(g["info"])
            grate = float((g["status"] == cb.IK_CONVERGED).mean())
            gms = statistics.median(gt)
            speedup = cpu_ms / gms
            rate_diff_pts = (rate - grate) * 100.0
            go_a, go_b = speedup >= 5.0, rate_diff_pts <= 1.0
            gpu_res = {
                "e2e_ms_median": gms, "e2e_ms_all": gt, "convergence_rate": grate,
                "status_counts": {int(k): int((g["status"] == k).sum()) for k in range(4)},
                "last_run_info": infos[-1],
                "speedup_vs_cpu": speedup, "rate_gap_points_cpu_minus_gpu": rate_diff_pts,
                "rule_a_speedup_ge_5x": bool(go_a), "rule_b_rate_within_1pt": bool(go_b),
                "verdict": "GO" if (go_a and go_b) else "NO-GO",
            }
    except ImportError as e:  # pragma: no cover
        gpu_res = {"skipped": f"gpu_bridge not importable: {e}"}

    out = {
        "benchmark": "r5_ik",
        "date": datetime.date.today().isoformat(),
        "n_targets": args.n, "repeats": args.repeats,
        "robot": "six_dof_fixture (six_dof_with_gripper.urdf, tcp l6)",
        "start": "q_true + U(-0.5,0.5)/sqrt(dof), clipped to limits",
        "versions": {
            "numpy": np.__version__, "python": platform.python_version(),
            "os": platform.platform(), "cpu": _cpu_name(), "cpu_threads": os.cpu_count(),
            "git_commit": _run(["git", "-C", str(ROOT), "rev-parse", "HEAD"]),
            "nvcc": _run("nvcc --version").splitlines()[-1:] or "unavailable",
            **gpu_ver,
        },
        "results": {
            "cpu_all_threads": {"e2e_ms_median": statistics.median(times), "e2e_ms_all": times,
                                "convergence_rate": rate, "status_counts": counts},
            "gpu_fp32_plus_polish": gpu_res,
        },
    }
    path = Path(args.out) if args.out else ROOT / "benchmarks" / f"r5_ik_{out['date']}.json"
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(out, indent=2))
    print(json.dumps(out["results"], indent=2))
    print(f"written {path}")


if __name__ == "__main__":
    main()
