"""Workspace analysis and data export of the API (R8.1)."""
from __future__ import annotations

import numpy as np

from ._cancel import check_cancel, current_token
from ._util import guard
from .errors import AnalysisError, ExportError, InvalidArgument, ScriptCancelled
from .robot import Robot


@guard(AnalysisError)
def workspace(robot: Robot, n_samples: int = 200_000, voxel: float | None = None,
              seed: int = 42, metrics: bool = False) -> dict:
    """Monte Carlo workspace of the TCP position (CPU).

    ``n_samples`` random joint vectors inside the joint limits (all limits must be finite);
    ``voxel``: voxel edge in m (default reach / 100); ``seed``: random seed; ``metrics=True``
    adds per-voxel singular-value and manipulability statistics. Returns a dict of numpy arrays
    and numbers: ``volume`` (m^3, voxel count x voxel^3), ``volume_err_bound`` (m^3), ``filled``
    (voxel count), ``voxel_size`` (m), ``origin`` (m, grid corner), ``dims`` (grid size),
    ``n_samples``, ``keys`` (linear voxel index), ``count`` (samples per filled voxel) and
    ``centers`` (filled, 3) voxel centres in the base frame (m). Cancel-aware.
    """
    from app.analysis.workspace_analysis import analyze_workspace, voxel_centers
    n = int(n_samples)
    if n < 1:
        raise InvalidArgument("n_samples must be >= 1")
    opts: dict = {"n_samples": n, "seed": int(seed), "want_metrics": bool(metrics)}
    if voxel is not None:
        if not (float(voxel) > 0):
            raise InvalidArgument("voxel must be > 0")
        opts["voxel_size"] = float(voxel)
    check_cancel()
    res = analyze_workspace(robot._model, opts, cancel=current_token())
    if res.get("cancelled"):
        check_cancel()
        raise ScriptCancelled("workspace cancelled")
    res["centers"] = voxel_centers(res)
    return res


def _arrays(result: dict, names) -> dict:
    """Recorder-style ``{name: (t, data)}`` from a ``simulate`` result dict."""
    if not isinstance(result, dict) or "t" not in result:
        raise InvalidArgument("result must be the dict returned by simulate (has key 't')")
    t = np.asarray(result["t"], dtype=float)
    keys = [k for k in result if k != "t"] if names is None else list(names)
    out = {}
    for k in keys:
        if k not in result:
            raise InvalidArgument(f"unknown signal {k!r}")
        d = np.asarray(result[k], dtype=float)
        if d.ndim == 1:
            d = d.reshape(-1, 1)
        if d.ndim != 2 or len(d) != len(t):
            raise InvalidArgument(f"signal {k!r} must have one row per time sample")
        out[k] = (t, d)
    return out


def _meta(meta: dict | None) -> dict:
    from app.sim.export import build_meta
    m = build_meta()
    m.update(dict(meta or {}))
    return m


@guard(ExportError)
def export_csv(result: dict, path: str, names: list[str] | None = None,
               meta: dict | None = None) -> int:
    """Write the ``simulate`` result to a CSV file (time column ``t [s]`` plus one column per
    signal entry, units in the header, metadata in ``#`` comment lines). Returns the row count."""
    from app.sim.export import export_csv as _csv
    return int(_csv(_arrays(result, names), str(path), None, _meta(meta)))


@guard(ExportError)
def export_npz(result: dict, path: str, names: list[str] | None = None,
               meta: dict | None = None) -> int:
    """Write the ``simulate`` result to a compressed NPZ file (``<name>_t``, ``<name>``, JSON
    ``meta``). Returns the row count."""
    from app.sim.export import export_npz as _npz
    return int(_npz(_arrays(result, names), str(path), _meta(meta), None))
