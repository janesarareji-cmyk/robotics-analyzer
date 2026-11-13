"""U6.2: reach map overlay. Voxels the TCP can reach, coloured by dexterity (mean manipulability).

Uses ``workspace_analysis.analyze_workspace`` (same sampling, voxel keys and counts) with the batch
metrics of ``batch_jacobian``. Optional slice: only the voxel layer at a chosen height (a table-height
map). Probe: the voxel under a point (the dragged target) is outlined, green = reachable.
"""
from __future__ import annotations

import time
from typing import Any

import numpy as np

from app.analysis import workspace_analysis as wa
from app.analysis.batch_jacobian import fast_sample_metrics
from app.analysis.overlay_colors import colormap_rgb, colormap_stops
from app.analysis.overlays import Overlay
from app.compute.backend import estimate_reach

# quality -> (samples, grid cells across the reach)
QUALITY = {"Fast": (150_000, 25), "Normal": (400_000, 40), "Fine": (1_000_000, 60)}
PROBE_OK = "#66bb6a"
PROBE_BAD = "#ff5252"


class ReachOverlay(Overlay):
    key = "reach"
    title = "Reach map"
    blurb = "Where the tool can reach, coloured by how freely it can move there (dexterity)."
    default_opacity = 0.35
    recompute_options = ("quality", "seed")

    def __init__(self) -> None:
        self._data: dict | None = None
        self._opts: dict = {}
        self._base = np.eye(4)

    def default_options(self) -> dict:
        return {"quality": "Normal", "seed": 42, "slice": False, "height": 0.5}

    # ------------------------------------------------------------------ compute (worker)
    def compute(self, model, opts, cancel, progress) -> dict:
        samples, cells = QUALITY.get(opts.get("quality", "Normal"), QUALITY["Normal"])
        h = max(estimate_reach(model) / cells, 1e-3)
        t0 = time.perf_counter()
        res = wa.analyze_workspace(
            model, dict(n_samples=samples, seed=int(opts.get("seed", 42)), voxel_size=h,
                        metric_mode="position"), cancel, progress, metrics_fn=fast_sample_metrics)
        res["seconds"] = time.perf_counter() - t0
        if res["filled"]:
            res["w_ref"] = float(max(np.percentile(res["mean_w"], 95), 1e-12))
        else:
            res["w_ref"] = 1.0
        return res

    # ------------------------------------------------------------------ drawing (GUI)
    @staticmethod
    def _layer(res: dict, height_frac: float) -> int:
        dims = res["dims"]
        return int(min(max(round(height_frac * (dims[2] - 1)), 0), dims[2] - 1))

    @classmethod
    def layer_mask(cls, res: dict, height_frac: float) -> np.ndarray:
        """Voxels of the z layer at ``height_frac`` (0..1) of the grid height."""
        return wa.voxel_index(res)[:, 2] == cls._layer(res, height_frac)

    @classmethod
    def slice_height(cls, res: dict, height_frac: float) -> float:
        """Base-frame z of the centre of the chosen layer."""
        return float(res["origin"][2] + (cls._layer(res, height_frac) + 0.5) * res["voxel_size"])

    def colors(self, res: dict) -> np.ndarray:
        return colormap_rgb("viridis", res["mean_w"] / res["w_ref"])

    def show(self, view, data, opts, base, opacity) -> int:
        self._data, self._opts, self._base = data, dict(opts), np.asarray(base, float)
        if data is None or data["filled"] == 0:
            view.clear_overlay(self.key)
            return 0
        centers = wa.voxel_centers(data)
        rgb = self.colors(data)
        if opts.get("slice"):
            m = self.layer_mask(data, float(opts.get("height", 0.5)))
            centers, rgb = centers[m], rgb[m]
        if len(centers) == 0:
            view.clear_overlay(self.key)
            return 0
        return view.set_overlay_voxels(self.key, centers, data["voxel_size"], rgb, opacity, base)

    def hide(self, view) -> None:
        view.clear_overlay(self.key, render=False)
        view.clear_overlay(self.key + "_probe")

    def legend(self, opts) -> list:
        return colormap_stops("viridis")

    def legend_labels(self, opts) -> tuple[str, str]:
        return ("hard to move", "free to move")

    def summary(self, data, opts) -> str:
        if data is None:
            return ""
        if data.get("message"):
            return data["message"]
        s = (f"{data['filled']} voxels of {data['voxel_size'] * 1000:.0f} mm, volume "
             f"{data['volume']:.3g} m3, {data.get('seconds', 0.0):.1f} s")
        if opts.get("slice"):
            s += f". Slice at z = {self.slice_height(data, float(opts.get('height', 0.5))):.2f} m"
        return s

    # ------------------------------------------------------------------ probe (live, GUI)
    def probe(self, view, point_world, base=None) -> bool | None:
        """Outline the voxel that contains ``point_world``; True when it is a reachable voxel."""
        res = self._data
        if res is None or res["filled"] == 0 or point_world is None:
            view.clear_overlay(self.key + "_probe")
            return None
        base = self._base if base is None else np.asarray(base, float)
        p = np.linalg.inv(base) @ np.append(np.asarray(point_world, float), 1.0)
        h = float(res["voxel_size"])
        ijk = np.floor((p[:3] - np.asarray(res["origin"])) / h).astype(int)
        v = wa.voxel_value(res, int(ijk[0]), int(ijk[1]), int(ijk[2]), "count")
        ok = bool(np.isfinite(v))
        centre = np.asarray(res["origin"]) + (ijk + 0.5) * h
        view.set_overlay_outline(self.key + "_probe", centre, h * 1.02,
                                 PROBE_OK if ok else PROBE_BAD, base)
        return ok

    # ------------------------------------------------------------------ extra controls
    def make_controls(self, manager) -> Any:
        from PySide6.QtCore import Qt
        from PySide6.QtWidgets import (QCheckBox, QComboBox, QHBoxLayout, QLabel, QSlider,
                                       QVBoxLayout, QWidget)
        w = QWidget()
        lay = QVBoxLayout(w)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(3)
        row = QHBoxLayout()
        row.addWidget(QLabel("Quality"))
        q = QComboBox()
        q.addItems(list(QUALITY))
        q.setCurrentText(manager.options(self.key)["quality"])
        q.setToolTip("Fast = coarse and quick, Fine = more samples and smaller cubes")
        q.setAccessibleName("Reach map quality")
        row.addWidget(q, 1)
        lay.addLayout(row)
        sl = QCheckBox("Slice at a height")
        sl.setToolTip("Show only one horizontal layer, a map for a table height")
        lay.addWidget(sl)
        hr = QHBoxLayout()
        slider = QSlider(Qt.Orientation.Horizontal)
        slider.setRange(0, 100)
        slider.setValue(50)
        slider.setEnabled(False)
        slider.setAccessibleName("Slice height")
        hlab = QLabel("z = -")
        hlab.setMinimumWidth(70)
        hr.addWidget(slider, 1)
        hr.addWidget(hlab)
        lay.addLayout(hr)
        q.currentTextChanged.connect(lambda t: manager.set_option(self.key, "quality", t))
        sl.toggled.connect(lambda on: (slider.setEnabled(on), manager.set_option(self.key, "slice", on)))
        slider.valueChanged.connect(lambda v: manager.set_option(self.key, "height", v / 100.0))

        def refresh(key: str) -> None:
            if key != self.key:
                return
            d = manager.data(self.key)
            hlab.setText("z = -" if d is None or d["filled"] == 0 else
                         f"z = {self.slice_height(d, slider.value() / 100.0):.2f} m")

        manager.changed.connect(refresh)
        w.slice_check, w.slice_slider, w.quality_combo = sl, slider, q
        return w
