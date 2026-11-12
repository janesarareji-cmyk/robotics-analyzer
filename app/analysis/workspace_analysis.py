"""R7.5: reachable volume, manipulability map and singularity regions from joint samples.

Samples q uniformly inside the joint limits (numpy ``default_rng(seed)``, chunked), computes
the TCP position in the BASE frame (``cb.fk_batch``) and, per sample, the Jacobian
(``cb.jacobian_tcp``) and its indicators (``cb.analyze_singularity``). Per voxel it keeps
count, sum/min of sigma_min and sum/min of Yoshikawa w, sparse (sorted voxel keys), so memory
is bounded by the number of occupied voxels, not by the grid.

Voxel key: ``lin = (ix * ny + iy) * nz + iz`` (same as the R5 grid, `docs/workspace-sampling.md`).
Volume = filled voxels x voxel_size^3. Error: every filled voxel that touches an empty voxel
(6-neighbourhood) may be only partly inside the true set, and unsampled interior voxels are
missed; ``volume_err_bound`` = boundary voxels x voxel_size^3 (see `docs/workspace-analysis.md`).
"""
from __future__ import annotations

from typing import Any, Callable

import numpy as np

import core_bridge as cb
from app.compute.backend import estimate_reach
from app.kin_utils import dof_limits

CHUNK = 20_000
METRIC_MODES = ("position", "full")
SLICE_FIELDS = ("mean_smin", "min_smin", "mean_w", "min_w", "count")
DEFAULT_OPTIONS = dict(n_samples=200_000, seed=42, voxel_size=0.0, metric_mode="position",
                       threshold=1e-2, singular_stat="min", want_metrics=True,
                       bounds_lo=None, bounds_hi=None, q=None)
EMPTY_MESSAGE = "No samples reached the grid: workspace is empty (check joint limits and voxel size)."


def _opts(options: dict | None) -> dict:
    o = dict(DEFAULT_OPTIONS)
    for k, v in (options or {}).items():
        if k not in DEFAULT_OPTIONS:
            raise ValueError(f"unknown option '{k}'")
        o[k] = v
    if o["metric_mode"] not in METRIC_MODES:
        raise ValueError(f"metric_mode must be one of {METRIC_MODES}")
    if o["singular_stat"] not in ("mean", "min"):
        raise ValueError("singular_stat must be 'mean' or 'min'")
    if o["q"] is None and int(o["n_samples"]) < 1:
        raise ValueError("n_samples must be >= 1")
    return o


def sample_metrics(cm, q: np.ndarray, mode: str = "position") -> tuple[np.ndarray, np.ndarray]:
    """Per-sample (sigma_min, Yoshikawa w) of the TCP Jacobian. ``q`` (N, n_dof).

    mode "position": rows 0-2 (linear velocity); planar 2R gives w = l1 l2 |sin q2|.
    """
    q = np.asarray(q, dtype=float)
    n = len(q)
    smin = np.empty(n)
    w = np.empty(n)
    ana = cb.analyze_singularity
    jac = cb.jacobian_tcp
    for i in range(n):
        s = ana(np.asarray(jac(cm, q[i].tolist())), mode)
        smin[i] = s["sigma_min"]
        w[i] = s["manipulability"]
    return smin, w


def _merge(state: dict | None, keys, cnt, ssum, smin_min, wsum, wmin) -> dict:
    new = dict(count=cnt, s_sum=ssum, s_min=smin_min, w_sum=wsum, w_min=wmin)
    if state is None:
        return dict(keys=keys, **new)
    allk = np.union1d(state["keys"], keys)
    n = len(allk)
    out = dict(keys=allk, count=np.zeros(n, np.int64), s_sum=np.zeros(n), s_min=np.full(n, np.inf),
               w_sum=np.zeros(n), w_min=np.full(n, np.inf))
    for src, ks in ((state, state["keys"]), (new, keys)):
        ix = np.searchsorted(allk, ks)
        out["count"][ix] += src["count"]
        out["s_sum"][ix] += src["s_sum"]
        out["w_sum"][ix] += src["w_sum"]
        out["s_min"][ix] = np.minimum(out["s_min"][ix], src["s_min"])
        out["w_min"][ix] = np.minimum(out["w_min"][ix], src["w_min"])
    return out


def _chunk_stats(lin: np.ndarray, smin, w):
    u, inv = np.unique(lin, return_inverse=True)
    inv = inv.reshape(-1)
    cnt = np.bincount(inv).astype(np.int64)
    if smin is None:
        return u, cnt, np.zeros(len(u)), np.full(len(u), np.inf), np.zeros(len(u)), np.full(len(u), np.inf)
    ssum = np.bincount(inv, weights=smin)
    wsum = np.bincount(inv, weights=w)
    smm = np.full(len(u), np.inf)
    wmm = np.full(len(u), np.inf)
    np.minimum.at(smm, inv, smin)
    np.minimum.at(wmm, inv, w)
    return u, cnt, ssum, smm, wsum, wmm


def _boundary_count(keys: np.ndarray, dims) -> int:
    nx, ny, nz = dims
    iz = keys % nz
    iy = (keys // nz) % ny
    ix = keys // (nz * ny)
    boundary = np.zeros(len(keys), dtype=bool)
    for axis, (idx, n) in enumerate(((ix, nx), (iy, ny), (iz, nz))):
        for d in (-1, 1):
            nb = idx + d
            inside = (nb >= 0) & (nb < n)
            c = [ix, iy, iz]
            c = [x.copy() for x in c]
            c[axis] = np.clip(nb, 0, n - 1)
            lin = (c[0] * ny + c[1]) * nz + c[2]
            pos = np.clip(np.searchsorted(keys, lin), 0, len(keys) - 1)
            boundary |= ~(inside & (keys[pos] == lin))
    return int(boundary.sum())


def analyze_workspace(model, options: dict | None = None, cancel=None,
                      progress: Callable[[int, int], Any] | None = None) -> dict:
    """Run the study. ``cancel`` has ``is_cancelled()`` (cb.CancelToken); ``progress(done, total)``.

    ``message`` is non-empty when the workspace is empty; ``cancelled`` True when stopped
    early (partial result). Options: see ``DEFAULT_OPTIONS`` and docs/workspace-analysis.md.
    """
    o = _opts(options)
    cm = cb.compile(model)
    n_dof = int(model.dof())
    lo, hi = dof_limits(model)
    lo_a, hi_a = np.asarray(lo, float), np.asarray(hi, float)
    if o["q"] is None and not (np.all(np.isfinite(lo_a)) and np.all(np.isfinite(hi_a))
                               and np.all(hi_a >= lo_a)):
        raise ValueError("joint limits must be finite with lower <= upper")
    h = float(o["voxel_size"])
    reach = estimate_reach(model)
    if h <= 0.0:
        h = max(reach / 100.0, 1e-4)
    if o["bounds_lo"] is not None:
        blo = np.asarray(o["bounds_lo"], float).reshape(3)
        bhi = np.asarray(o["bounds_hi"], float).reshape(3)
    else:
        blo = np.full(3, -(reach + h))
        bhi = np.full(3, reach + h)
    if np.any(bhi <= blo):
        raise ValueError("bounds_hi must be > bounds_lo")
    dims = tuple(int(np.ceil((bhi[a] - blo[a]) / h)) for a in range(3))
    if dims[0] * dims[1] * dims[2] > 2**62:
        raise ValueError("grid too large")
    _nx, ny, nz = dims
    q_all = None if o["q"] is None else np.asarray(o["q"], float).reshape(-1, n_dof)
    total = len(q_all) if q_all is not None else int(o["n_samples"])
    rng = np.random.default_rng(int(o["seed"]))
    metrics = bool(o["want_metrics"])
    state = None
    n_done = n_out = 0
    cancelled = False
    while n_done < total:
        if cancel is not None and cancel.is_cancelled():
            cancelled = True
            break
        m = min(CHUNK, total - n_done)
        if q_all is not None:
            q = q_all[n_done:n_done + m]
        else:
            q = lo_a + rng.random((m, n_dof)) * (hi_a - lo_a)
        pos = np.asarray(cb.fk_batch(cm, q, "base")["position"])
        idx = np.floor((pos - blo) / h).astype(np.int64)
        ok = np.all((idx >= 0) & (idx < np.array(dims)), axis=1)
        n_out += int(m - ok.sum())
        if ok.any():
            idx = idx[ok]
            lin = (idx[:, 0] * ny + idx[:, 1]) * nz + idx[:, 2]
            sm = wm = None
            if metrics:
                sm, wm = sample_metrics(cm, q[ok], o["metric_mode"])
            state = _merge(state, *_chunk_stats(lin, sm, wm))
        n_done += m
        if progress is not None:
            progress(n_done, total)
    res: dict = dict(origin=blo, voxel_size=h, dims=dims, n_samples=total, n_samples_done=n_done,
                     n_outside=n_out, cancelled=cancelled, metric_mode=o["metric_mode"],
                     has_metrics=metrics, threshold=float(o["threshold"]),
                     singular_stat=o["singular_stat"], message="")
    if state is None:
        state = dict(keys=np.zeros(0, np.int64), count=np.zeros(0, np.int64), s_sum=np.zeros(0),
                     s_min=np.zeros(0), w_sum=np.zeros(0), w_min=np.zeros(0))
    keys = state["keys"]
    c = np.maximum(state["count"], 1)
    res.update(keys=keys, count=state["count"], mean_smin=state["s_sum"] / c,
               min_smin=state["s_min"], mean_w=state["w_sum"] / c, min_w=state["w_min"])
    res["filled"] = int(len(keys))
    res["volume"] = res["filled"] * h**3
    res["volume_err_bound"] = (_boundary_count(keys, dims) * h**3) if len(keys) else 0.0
    if metrics:
        stat = res["mean_smin"] if o["singular_stat"] == "mean" else res["min_smin"]
        res["singular_mask"] = stat < float(o["threshold"])
    else:
        res["singular_mask"] = np.zeros(len(keys), dtype=bool)
    res["n_singular"] = int(res["singular_mask"].sum())
    if res["filled"] == 0 and not cancelled:
        res["message"] = EMPTY_MESSAGE
    return res


def volume_from_occupancy(res: dict) -> float:
    """Volume of an R5 ``sample_workspace`` result: filled voxels x voxel_size^3."""
    return len(res["occupied_voxels"]) * float(res["voxel_size"]) ** 3


def voxel_index(res: dict) -> np.ndarray:
    """(M, 3) integer (ix, iy, iz) of the filled voxels."""
    _nx, ny, nz = res["dims"]
    k = res["keys"]
    return np.stack([k // (nz * ny), (k // nz) % ny, k % nz], axis=1)


def voxel_centers(res: dict) -> np.ndarray:
    """(M, 3) centers of the filled voxels in the BASE frame."""
    return np.asarray(res["origin"]) + (voxel_index(res) + 0.5) * float(res["voxel_size"])


def field(res: dict, name: str) -> np.ndarray:
    """Per-voxel values (aligned with ``res['keys']``)."""
    if name == "singular":
        return res["singular_mask"].astype(float)
    if name not in SLICE_FIELDS:
        raise ValueError(f"unknown field '{name}'")
    if name != "count" and not res.get("has_metrics", True):
        raise ValueError("this result has no metrics (want_metrics=False)")
    return np.asarray(res[name], dtype=float)


def voxel_value(res: dict, ix: int, iy: int, iz: int, name: str) -> float:
    """Direct lookup of one voxel; NaN when the voxel is empty or outside the grid."""
    nx, ny, nz = res["dims"]
    if not (0 <= ix < nx and 0 <= iy < ny and 0 <= iz < nz):
        return float("nan")
    lin = (ix * ny + iy) * nz + iz
    k = res["keys"]
    p = int(np.searchsorted(k, lin))
    if p >= len(k) or k[p] != lin:
        return float("nan")
    return float(field(res, name)[p])


def slice_image(res: dict, name: str, axis: int, offset: float) -> dict:
    """2D slice of ``name`` at base-frame coordinate ``offset`` along ``axis`` (0=x,1=y,2=z).

    Returns dict(image (n_a, n_b) NaN where empty, axes (a, b), extent (a0, a1, b0, b1),
    layer, valid). ``image[ia, ib]`` is the voxel with index ``ia`` on axis a, ``ib`` on b.
    """
    if axis not in (0, 1, 2):
        raise ValueError("axis must be 0, 1 or 2")
    dims = res["dims"]
    h = float(res["voxel_size"])
    o = np.asarray(res["origin"], float)
    a, b = [d for d in range(3) if d != axis]
    layer = int(np.floor((float(offset) - o[axis]) / h))
    img = np.full((dims[a], dims[b]), np.nan)
    valid = 0 <= layer < dims[axis]
    if valid and len(res["keys"]):
        ijk = voxel_index(res)
        sel = ijk[:, axis] == layer
        img[ijk[sel, a], ijk[sel, b]] = field(res, name)[sel]
    extent = (o[a], o[a] + dims[a] * h, o[b], o[b] + dims[b] * h)
    return dict(image=img, axes=(a, b), extent=extent, layer=layer, valid=valid)
