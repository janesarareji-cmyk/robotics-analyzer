"""U6.4: joint load overlay. A torque bar and a speed bar next to every joint while the robot moves.

torque = RNEA (``cb.inverse_dynamics``) at the current q, qd, qdd; ratio = |tau| / effort limit.
speed ratio = |qd| / velocity limit. Slider, drag and jog give qd = qdd = 0, so the bar shows the
static gravity torque; kinematic playback gives qd, qdd from the trajectory. A limit of 0 or less means
"no limit" (ratio 0). Gripper DOFs are not drawn. Refresh at most 30 Hz.
"""
from __future__ import annotations

import time
from typing import Any

import numpy as np

import core_bridge as cb
from app.analysis.overlays import Overlay
from app.gripper import gripper_dofs
from app.kin_utils import dof_joint_types
from app.panels.model_panel import joint_links

MIN_INTERVAL = 1.0 / 30.0
DEFAULT_GRAVITY = (0.0, 0.0, -9.81)


def dof_masters(model: Any) -> list[int]:
    """Index of the master joint of every DOF."""
    master = [-1] * int(model.dof())
    for j_idx, q_idx in enumerate(model.q_index_map()):
        if q_idx >= 0:
            master[q_idx] = j_idx
    return master


def load_ratios(model: Any, dm: Any, q, qd=None, qdd=None, gravity=DEFAULT_GRAVITY) -> dict:
    """Torque and speed ratios per DOF. Keys: tau, effort, tratio, qd, vlimit, sratio, names."""
    n = int(model.dof())
    q = [float(v) for v in q]
    qd = [0.0] * n if qd is None else [float(v) for v in qd]
    qdd = [0.0] * n if qdd is None else [float(v) for v in qdd]
    tau = np.asarray(cb.inverse_dynamics(dm, q, qd, qdd, [float(v) for v in gravity]), float)
    masters = dof_masters(model)
    effort = np.array([float(model.joints[j].limits.effort) for j in masters])
    vlim = np.array([float(model.joints[j].limits.vel) for j in masters])
    qd_a = np.asarray(qd, float)
    with np.errstate(divide="ignore", invalid="ignore"):
        tr = np.where(effort > 0, np.abs(tau) / np.where(effort > 0, effort, 1.0), 0.0)
        sr = np.where(vlim > 0, np.abs(qd_a) / np.where(vlim > 0, vlim, 1.0), 0.0)
    names = [model.joints[j].name or f"joint{j + 1}" for j in masters]
    return dict(tau=tau, effort=effort, tratio=tr, qd=qd_a, vlimit=vlim, sratio=sr, names=names)


class LoadOverlay(Overlay):
    key = "load"
    title = "Joint load"
    blurb = ("A torque bar (T) and a speed bar (S) next to each joint while you move the robot: "
             "green below 70 % of the limit, orange up to 100 %, red above. At rest it shows the "
             "gravity load.")
    live = True
    default_opacity = 1.0

    def __init__(self) -> None:
        self.gravity_fn = None            # () -> (gx, gy, gz) or None
        self.bars: Any = None
        self.last: dict | None = None
        self.last_cost_ms = 0.0
        self.updates = 0
        self._last_t = 0.0
        self._dm: Any = None
        self._dm_model: Any = None
        self._pending: tuple | None = None
        self._observing = False
        self._active = False
        self._last_cam = 0.0

    def reset(self) -> None:
        self._dm = self._dm_model = None
        self.last = None
        self._pending = None

    def legend(self, opts) -> list:
        return [(0.0, (102, 187, 106)), (0.69, (102, 187, 106)), (0.7, (255, 152, 0)),
                (0.99, (255, 152, 0)), (1.0, (255, 48, 48))]

    def legend_labels(self, opts) -> tuple[str, str]:
        return ("0 - 70 % ok, 70 - 100 % high", "over 100 %")

    def summary(self, data, opts) -> str:
        return ""

    def hide(self, view) -> None:
        self._active = False
        if self.bars is not None:
            self.bars.set_items([])
            self.bars.hide()

    # ------------------------------------------------------------------
    def _dyn(self, model: Any) -> Any:
        if self._dm is None or self._dm_model is not model:
            self._dm = cb.build_dyn_model(model, cb.compile(model))
            self._dm_model = model
        return self._dm

    def _ensure_bars(self, view: Any) -> Any:
        if self.bars is None:
            from app.viewport.load_bars import LoadBars
            parent = view.widget.parentWidget() or view.widget
            self.bars = LoadBars(parent)
        self.bars.show()
        self.bars.raise_()
        if not self._observing:
            try:        # the bars follow a camera move (rotate, zoom) too
                iren = view._plotter.iren
                iren.add_observer("InteractionEvent", lambda *_a: self._on_camera(view))
                iren.add_observer("EndInteractionEvent", lambda *_a: self._on_camera(view, True))
                self._observing = True
            except Exception:
                pass
        self._active = True
        return self.bars

    def _on_camera(self, view, force: bool = False) -> None:
        now = time.perf_counter()
        if not self._active or self.last is None:
            return
        if force or now - self._last_cam >= MIN_INTERVAL:
            self._last_cam = now
            self.redraw_bars(view)

    @property
    def has_pending(self) -> bool:
        return self._pending is not None

    def update_live(self, model, view, q, qd, qdd, base, force=False) -> None:
        now = time.perf_counter()
        if not force and now - self._last_t < MIN_INTERVAL:
            self._pending = (model, view, q, qd, qdd, base)       # latest wins, drawn by flush()
            return
        self._last_t = now
        self._pending = None
        self._draw(model, view, q, qd, qdd)

    def flush(self) -> None:
        """Draw the newest state that the 30 Hz limit held back (called by a timer)."""
        if self._pending is not None and time.perf_counter() - self._last_t >= MIN_INTERVAL:
            model, view, q, qd, qdd, _b = self._pending
            self._pending = None
            self._last_t = time.perf_counter()
            self._draw(model, view, q, qd, qdd)

    def _draw(self, model, view, q, qd, qdd) -> None:
        t0 = time.perf_counter()
        grav = self.gravity_fn() if self.gravity_fn is not None else None
        data = load_ratios(model, self._dyn(model), q, qd, qdd, grav or DEFAULT_GRAVITY)
        fk = cb.fk(model)
        pairs = joint_links(model)
        masters = dof_masters(model)
        skip = {g.dof_index for g in gripper_dofs(model)}
        frames = np.asarray(fk["frames"], float)
        dofs = [d for d in range(len(masters)) if d not in skip and masters[d] >= 0]
        pts = np.array([frames[pairs[masters[d]][1]][:3, 3] for d in dofs]).reshape(-1, 3)
        data["dofs"] = dofs
        data["points"] = pts
        self.last_cost_ms = (time.perf_counter() - t0) * 1000.0
        data["cost_ms"] = self.last_cost_ms
        self.last = data
        self.updates += 1
        self.redraw_bars(view)

    def redraw_bars(self, view) -> None:
        """Project the stored joint points (also after a camera move) and paint."""
        d = self.last
        if d is None or view is None:
            return
        bars = self._ensure_bars(view)
        xy = view.project_points(d["points"])
        items = [(xy[i, 0], xy[i, 1], f"J{dof + 1}", float(d["tratio"][dof]), float(d["sratio"][dof]))
                 for i, dof in enumerate(d["dofs"])]
        bars.set_items(items)
