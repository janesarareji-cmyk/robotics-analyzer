"""U6.3: singularity zones overlay. Red = singular, orange = near singular, split by kind.

Sampling: q uniform in the joint limits, per sample the Jacobian (``batch_jacobian``, whole batches)
and its smallest singular value sigma_min. Per voxel the smallest sigma_min is kept per kind, so
the threshold slider and the kind check boxes redraw at once without a new compute.

Kinds (6+ DOF arms whose first and last three joints are revolute; every other robot is "general"):

-   wrist: the last three joint axes are nearly dependent (sigma_min of the 3 x 3 matrix of the axes < 0.2)
-   elbow: the arm is stretched or folded: distance shoulder point - wrist centre within 0.4 % of
    its maximum / minimum over the samples
-   shoulder: the wrist centre is close to the first joint axis (within 4 % of the reach above the
    smallest distance seen)
-   general: singular, none of the above

The wrist centre is the closest point of axis n-3 and axis n-2 (for a spherical wrist where all three
axes meet it is the wrist point; for an offset wrist like the UR arms it is the crossing of the two
axes that are fixed in the forearm), the shoulder point the closest point of axis 1 and axis 2.
The kind is a heuristic label of a singular sample: the sample is singular because sigma_min is small,
the kind only says why (first match in the order wrist, elbow, shoulder). Docs: docs/overlays.md.
"""
from __future__ import annotations

import csv
import time
from dataclasses import dataclass
from typing import Any

import numpy as np

import core_bridge as cb
from app.analysis import workspace_analysis as wa
from app.analysis.batch_jacobian import batch_pose_jacobian, sigma_min_batch
from app.analysis.overlay_colors import ORANGE, RED
from app.analysis.overlays import Overlay
from app.compute.backend import estimate_reach
from app.kin_utils import SINGULAR_WARN, dof_joint_types, dof_limits

KINDS = ("wrist", "elbow", "shoulder", "general")
WRIST_TOL = 0.2          # sigma_min of the wrist axes matrix
ELBOW_TOL = 0.004        # of the largest shoulder-wrist distance (about 9 deg of the elbow angle)
SHOULDER_TOL = 0.04      # of the reach
QUALITY = {"Fast": (200_000, 25), "Normal": (600_000, 40), "Fine": (2_000_000, 60)}
# the wrist kind exists at almost every position (any tool orientation can be a wrist singularity),
# so it is off at the start: the picture then shows the position dependent zones (elbow, shoulder)
DEFAULT_KINDS = ("elbow", "shoulder", "general")
THRESHOLD_MIN, THRESHOLD_MAX = 0.005, 0.3
CHUNK = 100_000


# ---------------------------------------------------------------------------- geometry helpers
def _cross(a, b):
    return np.cross(a, b)


def axis_lines(J: np.ndarray, p: np.ndarray, i: int):
    """(point (N,3), direction (N,3)) of the axis of revolute DOF ``i`` in the base frame."""
    z = J[:, 3:6, i]
    v = J[:, 0:3, i]
    return p + _cross(z, v), z


def closest_points(a1, z1, a2, z2):
    """Midpoint of the closest points of two lines per row, and the gap, and a parallel mask."""
    w0 = a1 - a2
    b = np.sum(z1 * z2, axis=1)
    d = np.sum(z1 * w0, axis=1)
    e = np.sum(z2 * w0, axis=1)
    den = 1.0 - b * b
    par = den < 1e-9
    den_s = np.where(par, 1.0, den)
    s = np.where(par, 0.0, (b * e - d) / den_s)
    t = np.where(par, e, (e - b * d) / den_s)
    p1 = a1 + s[:, None] * z1
    p2 = a2 + t[:, None] * z2
    return 0.5 * (p1 + p2), np.linalg.norm(p1 - p2, axis=1), par


@dataclass
class ArmInfo:
    has_kinds: bool = False
    wrist: str = "none"                 # "spherical" | "offset" | "none"
    shoulder: np.ndarray | None = None
    axis1: np.ndarray | None = None
    d_max: float = 1.0
    d_min: float = 0.0
    perp_min: float = 0.0
    reach: float = 1.0
    n: int = 0


def arm_info(model: Any, cm: Any, n_ref: int = 4000, seed: int = 0) -> ArmInfo:
    """Constants of the kind classification (from random poses; see the module text)."""
    n = int(model.dof())
    info = ArmInfo(n=n, reach=float(estimate_reach(model)))
    types = dof_joint_types(model)
    if n < 6 or types[0] != "revolute" or any(types[i] != "revolute" for i in (n - 3, n - 2, n - 1)):
        return info
    lo, hi = (np.asarray(v, float) for v in dof_limits(model))
    q = lo + np.random.default_rng(seed).random((n_ref, n)) * (hi - lo)
    J, p, _R = batch_pose_jacobian(cm, q)
    a0, z0 = axis_lines(J, p, 0)
    a1, z1 = axis_lines(J, p, 1)
    S, _g, par = closest_points(a0, z0, a1, z1)
    if par.all():
        return info
    info.shoulder = np.median(S[~par], axis=0)
    info.axis1 = np.median(z0, axis=0)
    info.axis1 /= max(np.linalg.norm(info.axis1), 1e-12)
    am, zm = axis_lines(J, p, n - 3)
    an, zn = axis_lines(J, p, n - 2)
    A, _g, parw = closest_points(am, zm, an, zn)
    ok = ~parw
    if not ok.any():
        return info
    rel = A[ok] - info.shoulder
    dist = np.linalg.norm(rel, axis=1)
    perp = np.linalg.norm(rel - (rel @ info.axis1)[:, None] * info.axis1, axis=1)
    info.d_max, info.d_min, info.perp_min = float(dist.max()), float(dist.min()), float(perp.min())
    al, zl = axis_lines(J, p, n - 1)
    gap = np.linalg.norm(_cross(A[ok] - al[ok], zl[ok]), axis=1)
    info.wrist = "spherical" if float(np.median(gap)) < 1e-3 else "offset"
    info.has_kinds = True
    return info


def classify(info: ArmInfo, J: np.ndarray, p: np.ndarray) -> np.ndarray:
    """Kind index (0 wrist, 1 elbow, 2 shoulder, 3 general) of every sample."""
    n_s = len(J)
    if not info.has_kinds:
        return np.full(n_s, 3, dtype=np.int8)
    n = info.n
    M = np.stack([J[:, 3:6, n - 3], J[:, 3:6, n - 2], J[:, 3:6, n - 1]], axis=2)     # (N, 3, 3)
    ev = np.linalg.eigvalsh(np.matmul(np.swapaxes(M, 1, 2), M))
    m_w = np.sqrt(np.maximum(ev[:, 0], 0.0)) / WRIST_TOL
    am, zm = axis_lines(J, p, n - 3)
    an, zn = axis_lines(J, p, n - 2)
    A, _g, par = closest_points(am, zm, an, zn)
    rel = A - info.shoulder
    dist = np.linalg.norm(rel, axis=1)
    perp = np.linalg.norm(rel - (rel @ info.axis1)[:, None] * info.axis1, axis=1)
    m_e = np.minimum(np.abs(dist - info.d_max), np.abs(dist - info.d_min)) / (ELBOW_TOL * info.d_max)
    m_s = (perp - info.perp_min) / (SHOULDER_TOL * info.reach)
    m_e = np.where(par, np.inf, m_e)
    m_s = np.where(par, np.inf, m_s)
    m = np.stack([m_w, m_e, m_s], axis=1)
    hit = m < 1.0                                   # priority wrist, elbow, shoulder
    kind = np.where(hit.any(axis=1), np.argmax(hit, axis=1), 3).astype(np.int8)
    return kind


# ---------------------------------------------------------------------------- the study
def study(model: Any, quality: str = "Normal", seed: int = 42, metric: str = "full",
          n_samples: int | None = None, voxel_size: float | None = None, cancel: Any = None,
          progress: Any = None, q_samples: Any = None) -> dict:
    """Voxel grid of the smallest sigma_min per kind. See the module text.

    ``q_samples`` (N, n) replaces the random samples (tests, a given set of poses).
    """
    cm = cb.compile(model)
    n = int(model.dof())
    lo, hi = (np.asarray(v, float) for v in dof_limits(model))
    samples, cells = QUALITY.get(quality, QUALITY["Normal"])
    if n_samples is not None:
        samples = int(n_samples)
    q_all = None if q_samples is None else np.asarray(q_samples, float).reshape(-1, n)
    if q_all is not None:
        samples = len(q_all)
    reach = float(estimate_reach(model))
    h = float(voxel_size) if voxel_size else max(reach / cells, 1e-3)
    blo = np.full(3, -(reach + h))
    dims = tuple(int(np.ceil(2.0 * (reach + h) / h)) for _ in range(3))
    _nx, ny, nz = dims
    info = arm_info(model, cm)
    rng = np.random.default_rng(int(seed))
    t0 = time.perf_counter()
    state: dict | None = None
    done = 0
    cancelled = False
    mode = "position" if metric == "position" else "full"
    while done < samples:
        if cancel is not None and cancel.is_cancelled():
            cancelled = True
            break
        m = min(CHUNK, samples - done)
        q = q_all[done:done + m] if q_all is not None else lo + rng.random((m, n)) * (hi - lo)
        J, p, _R = batch_pose_jacobian(cm, q)
        smin = sigma_min_batch(J, mode)
        kind = classify(info, J, p)
        idx = np.floor((p - blo) / h).astype(np.int64)
        ok = np.all((idx >= 0) & (idx < np.array(dims)), axis=1)
        if ok.any():
            idx, smin_o, kind_o = idx[ok], smin[ok], kind[ok]
            lin = (idx[:, 0] * ny + idx[:, 1]) * nz + idx[:, 2]
            u, inv = np.unique(lin, return_inverse=True)
            inv = inv.reshape(-1)
            arr = np.full((len(u), len(KINDS)), np.inf)
            for k in range(len(KINDS)):
                sel = kind_o == k
                if sel.any():
                    np.minimum.at(arr[:, k], inv[sel], smin_o[sel])
            cnt = np.bincount(inv).astype(np.int64)
            state = _merge(state, u, arr, cnt)
        done += m
        if progress is not None:
            progress(done, samples)
    seconds = time.perf_counter() - t0
    if state is None:
        state = dict(keys=np.zeros(0, np.int64), smin=np.zeros((0, len(KINDS))), count=np.zeros(0, np.int64))
    return dict(keys=state["keys"], smin_kind=state["smin"], count=state["count"], origin=blo,
                voxel_size=h, dims=dims, n_samples=samples, n_samples_done=done, cancelled=cancelled,
                seconds=seconds, metric=mode, wrist=info.wrist, has_kinds=info.has_kinds,
                filled=int(len(state["keys"])), message="" if len(state["keys"]) or cancelled
                else "No samples reached the grid.")


def _merge(state, keys, arr, cnt):
    if state is None:
        return dict(keys=keys, smin=arr, count=cnt)
    allk = np.union1d(state["keys"], keys)
    out = dict(keys=allk, smin=np.full((len(allk), arr.shape[1]), np.inf),
               count=np.zeros(len(allk), np.int64))
    for ks, a, c in ((state["keys"], state["smin"], state["count"]), (keys, arr, cnt)):
        ix = np.searchsorted(allk, ks)
        out["smin"][ix] = np.minimum(out["smin"][ix], a)
        out["count"][ix] += c
    return out


def zones(data: dict, threshold: float, kinds=KINDS) -> dict:
    """Which voxels are red / orange for ``threshold`` and the enabled ``kinds``.

    Returns dict(red, orange, sigma (min over the enabled kinds), kind (index of the kind with the
    smallest sigma)); the masks are aligned with ``data['keys']``.
    """
    enabled = [i for i, k in enumerate(KINDS) if k in kinds]
    sk = data["smin_kind"]
    if not enabled or len(sk) == 0:
        z = np.zeros(len(sk), bool)
        return dict(red=z, orange=z.copy(), sigma=np.full(len(sk), np.inf), kind=np.zeros(len(sk), int))
    sub = sk[:, enabled]
    sigma = sub.min(axis=1)
    kind = np.asarray(enabled)[np.argmin(sub, axis=1)]
    red = sigma < threshold
    orange = (sigma < 2.0 * threshold) & ~red
    return dict(red=red, orange=orange, sigma=sigma, kind=kind)


def export_csv(path: str, data: dict, threshold: float, kinds=KINDS, base: np.ndarray | None = None) -> int:
    """Write x, y, z (world, m), sigma_min, kind of every red or orange voxel. Returns the row count."""
    z = zones(data, threshold, kinds)
    sel = z["red"] | z["orange"]
    centers = wa.voxel_centers(data)[sel]
    if base is not None:
        b = np.asarray(base, float)
        centers = centers @ b[:3, :3].T + b[:3, 3]
    with open(path, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["x", "y", "z", "sigma_min", "kind"])
        for c, s, k in zip(centers, z["sigma"][sel], z["kind"][sel]):
            w.writerow([f"{c[0]:.6f}", f"{c[1]:.6f}", f"{c[2]:.6f}", f"{s:.6g}", KINDS[int(k)]])
    return int(sel.sum())


class SingularityOverlay(Overlay):
    key = "singularity"
    title = "Singularity zones"
    blurb = ("Red: the arm is singular here (it loses a direction of motion). Orange: close to it. "
             "The threshold is the same as the bars in the Joints and Matrices panels.")
    default_opacity = 0.45
    recompute_options = ("quality", "seed", "metric")

    def __init__(self) -> None:
        self._data: dict | None = None
        self._base = np.eye(4)
        self._opts: dict = {}

    def default_options(self) -> dict:
        return {"quality": "Normal", "seed": 42, "metric": "full", "threshold": SINGULAR_WARN,
                "kinds": DEFAULT_KINDS}

    def compute(self, model, opts, cancel, progress) -> dict:
        return study(model, opts.get("quality", "Normal"), int(opts.get("seed", 42)),
                     opts.get("metric", "full"), cancel=cancel, progress=progress)

    @property
    def data(self) -> dict | None:
        return self._data

    def show(self, view, data, opts, base, opacity) -> int:
        self._data, self._base, self._opts = data, np.asarray(base, float), dict(opts)
        if data is None or data["filled"] == 0:
            view.clear_overlay(self.key)
            return 0
        z = zones(data, float(opts.get("threshold", SINGULAR_WARN)), tuple(opts.get("kinds", KINDS)))
        sel = z["red"] | z["orange"]
        if not sel.any():
            view.clear_overlay(self.key)
            return 0
        centers = wa.voxel_centers(data)[sel]
        rgb = np.where(z["red"][sel][:, None], np.array(RED, np.uint8), np.array(ORANGE, np.uint8))
        return view.set_overlay_voxels(self.key, centers, data["voxel_size"], rgb, opacity, base)

    def legend(self, opts) -> list:
        return [(0.0, RED), (0.49, RED), (0.5, ORANGE), (1.0, ORANGE)]

    def legend_labels(self, opts) -> tuple[str, str]:
        thr = float(opts.get("threshold", SINGULAR_WARN))
        return (f"red: sigma_min < {thr:.3f}", f"orange: < {2 * thr:.3f}")

    def summary(self, data, opts) -> str:
        if data is None:
            return ""
        if data.get("message"):
            return data["message"]
        z = zones(data, float(opts.get("threshold", SINGULAR_WARN)), tuple(opts.get("kinds", KINDS)))
        kinds = "wrist, elbow, shoulder, general" if data["has_kinds"] else "general only"
        return (f"{int(z['red'].sum())} red, {int(z['orange'].sum())} orange of {data['filled']} voxels "
                f"({kinds}), {data['seconds']:.1f} s")

    # ------------------------------------------------------------------ controls
    def export(self, path: str) -> int:
        """CSV of the voxels currently shown (needs data)."""
        if self._data is None:
            raise ValueError("No singularity result yet: switch the overlay on first.")
        return export_csv(path, self._data, float(self._opts.get("threshold", SINGULAR_WARN)),
                          tuple(self._opts.get("kinds", KINDS)), self._base)

    def make_controls(self, manager) -> Any:
        from PySide6.QtCore import Qt
        from PySide6.QtWidgets import (QCheckBox, QComboBox, QFileDialog, QHBoxLayout, QLabel,
                                       QPushButton, QSlider, QVBoxLayout, QWidget)
        w = QWidget()
        lay = QVBoxLayout(w)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(3)
        row = QHBoxLayout()
        row.addWidget(QLabel("Quality"))
        q = QComboBox()
        q.addItems(list(QUALITY))
        q.setCurrentText(manager.options(self.key)["quality"])
        q.setAccessibleName("Singularity quality")
        q.setToolTip("Fast = 200 000 samples, Normal = 600 000, Fine = 2 million")
        row.addWidget(q, 1)
        lay.addLayout(row)

        trow = QHBoxLayout()
        trow.addWidget(QLabel("Threshold"))
        slider = QSlider(Qt.Orientation.Horizontal)
        slider.setRange(0, 1000)
        slider.setAccessibleName("Singularity threshold")
        slider.setToolTip("Smallest singular value of the Jacobian that still counts as singular "
                          "(log scale). The default is the value used by the other panels.")
        tlab = QLabel("")
        tlab.setMinimumWidth(52)
        reset = QPushButton("Reset")
        reset.setToolTip("Back to the standard threshold")
        trow.addWidget(slider, 1)
        trow.addWidget(tlab)
        trow.addWidget(reset)
        lay.addLayout(trow)

        def to_slider(thr: float) -> int:
            return int(round(1000 * np.log(thr / THRESHOLD_MIN) / np.log(THRESHOLD_MAX / THRESHOLD_MIN)))

        def from_slider(v: int) -> float:
            return float(THRESHOLD_MIN * (THRESHOLD_MAX / THRESHOLD_MIN) ** (v / 1000.0))

        slider.setValue(to_slider(float(manager.options(self.key)["threshold"])))
        tlab.setText(f"{from_slider(slider.value()):.3f}")

        def on_slider(v: int) -> None:
            thr = from_slider(v)
            tlab.setText(f"{thr:.3f}")
            manager.set_option(self.key, "threshold", thr)

        slider.valueChanged.connect(on_slider)
        reset.clicked.connect(lambda: slider.setValue(to_slider(SINGULAR_WARN)))

        krow = QHBoxLayout()
        boxes = {}
        for k in KINDS:
            cb_ = QCheckBox(k.capitalize())
            cb_.setChecked(k in manager.options(self.key)["kinds"])
            cb_.setToolTip({"wrist": "Last three joint axes nearly dependent (exists at almost every position)",
                            "elbow": "Arm fully stretched or folded",
                            "shoulder": "Wrist centre close to the first joint axis",
                            "general": "Singular for another reason"}[k])
            boxes[k] = cb_
            krow.addWidget(cb_)
        lay.addLayout(krow)

        def on_kinds() -> None:
            manager.set_option(self.key, "kinds", tuple(k for k, c in boxes.items() if c.isChecked()))

        for c in boxes.values():
            c.toggled.connect(on_kinds)
        q.currentTextChanged.connect(lambda t: manager.set_option(self.key, "quality", t))

        exp = QPushButton("Export CSV...")
        exp.setToolTip("Write x, y, z, sigma_min and kind of the red and orange voxels")
        lay.addWidget(exp)
        note = QLabel("")
        note.setWordWrap(True)
        lay.addWidget(note)

        def on_export() -> None:
            if self._data is None:
                note.setText("Switch the overlay on first.")
                return
            path, _ = QFileDialog.getSaveFileName(w, "Export singularity voxels", "singularity.csv",
                                                  "CSV (*.csv)")
            if path:
                note.setText(f"{self.export(path)} voxels written to {path}")

        exp.clicked.connect(on_export)

        def refresh(key: str) -> None:
            if key == self.key:
                has = self._data is not None and self._data["has_kinds"]
                for k, c in boxes.items():
                    if k != "general":
                        c.setEnabled(bool(has))
                exp.setEnabled(self._data is not None)

        manager.changed.connect(refresh)
        w.threshold_slider, w.kind_boxes, w.export_button, w.quality_combo = slider, boxes, exp, q
        refresh(self.key)
        return w
