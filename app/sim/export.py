"""Data export of recorder arrays (R6.7). No Qt dependency.

``arrays`` is ``Recorder.export_arrays()``: ``{name: (t (N,), data (N, w))}``.
CSV: joined table on the union of all time stamps, empty cell = signal has no sample there.
NPZ: ``<name>_t``, ``<name>`` and a JSON string ``meta`` (``meta["names"]`` = signal order).
Floats are written with ``%.17g`` (exact round trip).
"""
from __future__ import annotations

import datetime as _dt
import json
import re
import os
import subprocess

import numpy as np

from .recorder import SIGNALS

APP_VERSION = "0.1.0"
_UNITS = {name: unit for name, _w, unit, _s in SIGNALS}
_SUFFIX = {"tcp_pos": "xyz", "tcp_vel": "xyz", "tcp_quat": "wxyz"}
_CHUNK = 20000


def git_commit():
    """Short git commit of the app checkout, or None."""
    try:
        here = os.path.dirname(os.path.abspath(__file__))
        out = subprocess.run(["git", "rev-parse", "--short", "HEAD"], cwd=here, capture_output=True,
                             text=True, timeout=5)
        return out.stdout.strip() or None if out.returncode == 0 else None
    except Exception:  # noqa: BLE001
        return None


def build_meta(recorder=None, robot=None, joint_names=None, joint_types=None, **extra):
    """Metadata dict from a recorder/engine (all optional)."""
    meta = {"robot": robot or "", "date": _dt.datetime.now().isoformat(timespec="seconds"),
            "app version": APP_VERSION}
    eng = getattr(recorder, "engine", None)
    if eng is not None:
        meta["integrator"] = getattr(getattr(eng, "settings", None), "integrator", "")
        meta["dt"] = float(eng.dt)
        meta["gravity"] = [float(g) for g in eng.gravity]
    gc = git_commit()
    if gc:
        meta["git commit"] = gc
    if joint_names:
        meta["joint_names"] = list(joint_names)
    if joint_types:
        meta["joint_types"] = [str(t) for t in joint_types]
    meta.update(extra)
    return meta


def _check(arrays, names):
    if not arrays:
        raise ValueError("nothing to export: no signals")
    if names is None:
        names = list(arrays)
    else:
        names = list(names)
        bad = [n for n in names if n not in arrays]
        if bad:
            raise ValueError(f"unknown signal(s): {bad}")
        if not names:
            raise ValueError("nothing to export: empty names filter")
    if all(len(arrays[n][0]) == 0 for n in names):
        raise ValueError("nothing to export: recorder is empty")
    return names


def _unit(name, i, meta):
    u = (meta.get("units") or {}).get(name)
    if u is None:
        u = _UNITS.get(name, "")
    if "|" in u:
        types = meta.get("joint_types") or []
        pris = i < len(types) and str(types[i]).lower().startswith("p")
        a, b = u.split("|", 1)
        u = b if pris else a
    return u


def _col_labels(name, w, meta):
    if w == 1:
        return [f"{name} [{_unit(name, 0, meta)}]"]
    suf = _SUFFIX.get(name)
    return [f"{name}{suf[i] if suf and len(suf) == w else i + 1} [{_unit(name, i, meta)}]"
            for i in range(w)]


def _fmt(v):
    return "%.17g" % v


def export_csv(recorder_arrays, path, names=None, meta=None):
    """Write one joined CSV. Returns the number of rows."""
    def _one_line(v):
        return v.replace(chr(13), " ").replace(chr(10), " ") if isinstance(v, str) else v

    meta = {_one_line(str(k)): _one_line(v) for k, v in dict(meta or {}).items()}
    names = _check(recorder_arrays, names)
    ts = [np.asarray(recorder_arrays[n][0], float) for n in names]
    ds = [np.asarray(recorder_arrays[n][1], float).reshape(len(t), -1) for n, t in zip(names, ts)]
    same = all(len(t) == len(ts[0]) and np.array_equal(t, ts[0]) for t in ts)
    if same:
        tt = ts[0]
        idx = [None] * len(names)
    else:
        # one row per distinct time, but a time repeated inside one signal keeps its repeats
        uniq = np.unique(np.concatenate(ts))
        need = np.zeros(len(uniq), int)
        occ = []
        for t in ts:
            o = np.argsort(t, kind="stable")
            st = t[o]
            g = np.searchsorted(uniq, st)
            k = np.arange(len(st)) - np.searchsorted(st, st, side="left")
            need = np.maximum(need, np.bincount(g, weights=None, minlength=len(uniq)))
            occ.append((o, g, k))
        start = np.concatenate([[0], np.cumsum(need)[:-1]]).astype(int)
        tt = np.repeat(uniq, need)
        idx = []
        for t, (o, g, k) in zip(ts, occ):
            ix = np.empty(len(t), int)
            ix[o] = start[g] + k
            idx.append(ix)
    widths = [d.shape[1] for d in ds]
    labels = ["t [s]"]
    for n, w in zip(names, widths):
        labels += _col_labels(n, w, meta)
    with open(path, "w", newline="", encoding="utf-8") as f:
        for k, v in meta.items():
            f.write(f"# {k}: {json.dumps(v) if not isinstance(v, str) else v}\n")
        f.write("# columns: " + json.dumps({n: w for n, w in zip(names, widths)}) + "\n")
        f.write(",".join(labels) + "\n")
        n_rows = len(tt)
        for a in range(0, n_rows, _CHUNK):
            b = min(n_rows, a + _CHUNK)
            blk = np.full((b - a, 1 + sum(widths)), np.nan)
            have = np.zeros((b - a, 1 + sum(widths)), bool)
            blk[:, 0], have[:, 0] = tt[a:b], True
            c = 1
            for d, ix, w in zip(ds, idx, widths):
                if ix is None:
                    blk[:, c:c + w], have[:, c:c + w] = d[a:b], True
                else:
                    sel = (ix >= a) & (ix < b)
                    blk[ix[sel] - a, c:c + w] = d[sel]
                    have[ix[sel] - a, c:c + w] = True
                c += w
            f.write("\n".join(",".join(_fmt(v) if h else "" for v, h in zip(r, hr))
                              for r, hr in zip(blk.tolist(), have.tolist())) + "\n")
    return len(tt)


def export_npz(recorder_arrays, path, meta=None, names=None):
    """Write compressed NPZ: ``<name>_t``, ``<name>``, ``meta`` (JSON string)."""
    names = _check(recorder_arrays, names)
    meta = dict(meta or {})
    meta["names"] = names
    out = {}
    for n in names:
        t, d = recorder_arrays[n]
        t = np.asarray(t, float)
        out[f"{n}_t"] = t
        out[n] = np.asarray(d, float).reshape(len(t), -1)
    out["meta"] = np.array(json.dumps(meta))
    np.savez_compressed(path, **out)
    return len(next(iter(out.values())))


def load_npz(path):
    """``(arrays, meta)`` as written by ``export_npz``."""
    with np.load(path, allow_pickle=False) as z:
        meta = json.loads(str(z["meta"]))
        arrays = {n: (z[f"{n}_t"], z[n]) for n in meta["names"]}
    return arrays, meta


def load_csv(path):
    """``(arrays, meta)`` as written by ``export_csv``. Empty cells = no sample."""
    meta, widths, header, rows = {}, None, None, []
    with open(path, encoding="utf-8") as f:
        for line in f:
            line = line.rstrip("\n")
            if line.startswith("#"):
                k, _, v = line[1:].strip().partition(": ")
                if k == "columns":
                    widths = json.loads(v)
                else:
                    try:
                        meta[k] = json.loads(v) if v[:1] in "[{" else v
                    except ValueError:
                        meta[k] = v
            elif header is None:
                header = line.split(",")
            elif line:
                rows.append(line.split(","))
    if header is None or widths is None:
        raise ValueError("not an export CSV (missing header or columns line)")
    n = len(rows)
    vals = np.full((n, len(header)), np.nan)
    have = np.zeros((n, len(header)), bool)
    for i, r in enumerate(rows):
        for j, s in enumerate(r):
            if s != "":
                vals[i, j], have[i, j] = float(s), True
    t_all, arrays, c = vals[:, 0], {}, 1
    for name, w in widths.items():
        ok = have[:, c:c + w].all(axis=1)
        arrays[name] = (t_all[ok].copy(), vals[ok, c:c + w].copy())
        c += w
    meta["columns"] = header
    return arrays, meta


def arrays_from_series(series):
    """Graph series (``app.graphs.series.Series``) -> ``(arrays, units)``."""
    arrays, units = {}, {}
    for s in series:
        arrays[s.name] = (np.asarray(s.t, float), np.asarray(s.y, float).reshape(-1, 1))
        units[s.name] = s.unit.split("|")[0]
    return arrays, units
