"""Run the built exe with a scrubbed environment (no Python, no repo on PATH). Prints the result.

Usage: python packaging/scrub_smoke.py [dist_dir] [--cpu]
"""
import os
import subprocess
import sys
from pathlib import Path


def scrubbed_env(force_cpu: bool) -> dict:
    sr = os.environ.get("SystemRoot", r"C:\Windows")
    env = {"SystemRoot": sr, "TEMP": os.environ.get("TEMP", sr + r"\Temp"),
           "PATH": sr + r"\System32;" + sr, "QT_QPA_PLATFORM": "offscreen"}
    env["TMP"] = env["TEMP"]
    if force_cpu:
        env["ANALYZER_FORCE_CPU"] = "1"
    return env


def run_smoke(dist: Path, force_cpu: bool, timeout: int = 180):
    exe = dist / "RoboticsAnalyzer.exe"
    return subprocess.run([str(exe), "--smoke"], env=scrubbed_env(force_cpu), cwd=str(dist),
                          capture_output=True, text=True, timeout=timeout)


if __name__ == "__main__":
    d = Path(next((a for a in sys.argv[1:] if not a.startswith("--")), "dist/RoboticsAnalyzer"))
    r = run_smoke(d.resolve(), "--cpu" in sys.argv)
    print(r.returncode, r.stdout.strip(), r.stderr.strip()[-500:])
    sys.exit(r.returncode)
