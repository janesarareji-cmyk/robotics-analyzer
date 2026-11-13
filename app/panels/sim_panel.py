"""
app/panels/sim_panel.py
R6.3: "Simulation" dock. Start / Pause / Resume / Stop, mode (kinematic / dynamic),
integrator, dt, time scale, per-joint torque entries, gravity / friction on-off and a
status line. A QTimer (1000 / render_framerate ms) calls ``engine.advance(wall_dt)``
while RUNNING and emits ``state_updated(SimState)``; MainWindow pushes q to the
viewport and the joint panel. No long loop runs on the UI thread.
"""

from __future__ import annotations

import time
from typing import Any

import numpy as np
from PySide6.QtCore import QTimer, Signal
from PySide6.QtWidgets import (
    QCheckBox, QComboBox, QDoubleSpinBox, QFormLayout, QGridLayout, QGroupBox, QHBoxLayout, QLabel,
    QPushButton, QVBoxLayout, QWidget,
)

from app.kin_utils import current_q, dof_limits
from app.panels.timeline_panel import TimelinePanel
from app.sim import Recorder, SimEngine, make_integrator
from app.sim import mode as sim_mode
from app.sim.engine import PAUSED, RUNNING, STOPPED
from app.sim.playback import TrajectoryPlayer, resolve_gains

MAX_WALL_DT = 0.1   # s; a stalled UI never makes a huge catch-up step
DEFAULT_FPS = 60.0


class SimPanel(QWidget):
    state_updated = Signal(object)   # SimState; q already final for this tick
    collision_stopped = Signal(float, str)   # U3.4: playback stopped at t (s) by a collision
    log_requested = Signal(str)
    trajectory_changed = Signal(object)   # Trajectory or None (relay of the timeline)
    model_replaced = Signal()             # engine / recorder rebuilt (R6.6 graphs)
    position_changed = Signal(float)      # playback / seek time (R6.6 graph cursor)
    playback_finished = Signal()          # U4b.4: a trajectory played to its end
    transport_changed = Signal()          # U6.5: play / pause state or trajectory changed

    def __init__(self, parent: QWidget | None = None):
        super().__init__(parent)
        self._bg_play = False
        self._model: Any = None
        self._settings: Any = None
        self._engine: SimEngine | None = None
        self._base_friction: Any = None
        self._mode = sim_mode.KINEMATIC
        self._tau_boxes: list[QDoubleSpinBox] = []
        self._samples: list[tuple[float, np.ndarray]] = []
        self._est_tau = None
        self._player: TrajectoryPlayer | None = None
        self._collision_check = None
        self._recorder: Recorder | None = None
        self._dof_names: list[str] = []
        self._last_tick = 0.0
        self._timer = QTimer(self)
        self._timer.timeout.connect(self._on_tick)
        self._build_ui()
        self._update_controls()

    # ------------------------------------------------------------------ UI
    def _build_ui(self) -> None:
        lay = QVBoxLayout(self)
        self._timeline = TimelinePanel()
        lay.addWidget(self._timeline.detach_teach_group())   # U2-fix: Teach group on top
        row = QGridLayout()
        self._btn_start = QPushButton("Start")
        self._btn_pause = QPushButton("Pause")
        self._btn_resume = QPushButton("Resume")
        self._btn_stop = QPushButton("Stop")
        for i, b in enumerate((self._btn_start, self._btn_pause, self._btn_resume, self._btn_stop)):
            row.addWidget(b, i // 2, i % 2)
        lay.addLayout(row)
        self._btn_start.clicked.connect(self.start)
        self._btn_pause.clicked.connect(self.pause)
        self._btn_resume.clicked.connect(self.resume)
        self._btn_stop.clicked.connect(self.stop)

        form = QFormLayout()
        self._mode_combo = QComboBox()
        self._mode_combo.addItems(list(sim_mode.MODES))
        self._integ_combo = QComboBox()
        self._integ_combo.addItems(["euler", "rk4", "rk45"])
        self._integ_combo.setCurrentText("rk4")
        self._dt = QDoubleSpinBox()
        self._dt.setDecimals(5)
        self._dt.setRange(1e-5, 0.1)
        self._dt.setSingleStep(0.0005)
        self._dt.setValue(0.001)
        self._dt.setSuffix(" s")
        self._scale = QDoubleSpinBox()
        self._scale.setDecimals(2)
        self._scale.setRange(0.01, 100.0)
        self._scale.setValue(1.0)
        self._gravity = QCheckBox("Gravity on")
        self._gravity.setChecked(True)
        self._friction = QCheckBox("Friction on")
        self._friction.setChecked(True)
        self._estimate = QCheckBox("Estimate torque")
        self._estimate.setToolTip("Estimate torque in kinematic mode")
        form.addRow("Mode", self._mode_combo)
        form.addRow("Integrator", self._integ_combo)
        form.addRow("dt", self._dt)
        form.addRow("Time scale", self._scale)
        form.addRow(self._gravity)
        form.addRow(self._friction)
        form.addRow(self._estimate)
        lay.addLayout(form)
        self._mode_combo.currentTextChanged.connect(self._on_mode_changed)
        self._integ_combo.currentTextChanged.connect(self._on_integrator_changed)
        self._dt.valueChanged.connect(self._on_dt_changed)
        self._scale.valueChanged.connect(self._on_scale_changed)
        self._gravity.toggled.connect(self._on_env_changed)
        self._friction.toggled.connect(self._on_env_changed)

        self._tau_group = QGroupBox("Applied torque (Nm / N)")
        self._tau_form = QFormLayout(self._tau_group)
        lay.addWidget(self._tau_group)
        self._est_label = QLabel("")
        lay.addWidget(self._est_label)
        self._status = QLabel("No robot")
        self._status.setWordWrap(True)
        lay.addWidget(self._status)
        lay.addWidget(self._timeline)
        lay.addStretch(1)
        tl = self._timeline
        tl.set_q_provider(lambda: current_q(self._model) if self._model is not None else None)
        tl.trajectory_changed.connect(self._on_traj_changed)
        tl.play_requested.connect(self.play_trajectory)
        tl.pause_requested.connect(self.pause_trajectory)
        tl.stop_requested.connect(self.stop_trajectory)
        tl.seek_requested.connect(self.seek_trajectory)
        tl.speed_changed.connect(self._on_traj_speed)
        tl.loop_changed.connect(self._on_traj_loop)
        tl.log_requested.connect(self.log_requested)

    # ----------------------------------------------------------- properties
    @property
    def engine(self) -> SimEngine | None:
        return self._engine

    @property
    def recorder(self) -> Recorder | None:
        return self._recorder

    @property
    def dof_names(self) -> list[str]:
        return list(self._dof_names)

    @property
    def dof_types(self) -> list[str]:
        """"revolute" / "prismatic" per DOF (units of the graphs and exports)."""
        from app.kin_utils import dof_joint_types
        if self._engine is None or self._model is None:
            return []
        return dof_joint_types(self._model)

    @property
    def mode(self) -> str:
        return self._mode

    @property
    def timeline(self) -> TimelinePanel:
        return self._timeline

    @property
    def player(self) -> TrajectoryPlayer | None:
        return self._player

    @property
    def trajectory(self):
        """Last valid trajectory (stored in the project)."""
        return self._timeline.trajectory

    def set_trajectory(self, traj) -> bool:
        """Show a trajectory loaded from the project (silent). False = dropped (DOF)."""
        self._discard_player()
        ok = self._timeline.set_trajectory(traj)
        self._update_controls()
        return ok

    def load_trajectory(self, traj) -> bool:
        """Show a trajectory made in the app (e.g. planned): like set_trajectory, but the
        change is reported through ``trajectory_changed`` (project update)."""
        self._discard_player()
        ok = self._timeline.load_trajectory(traj)
        self._update_controls()
        return ok

    def clear_trajectory(self) -> None:
        self._discard_player()
        self._timeline.clear()
        self._update_controls()

    @property
    def timer(self) -> QTimer:
        return self._timer

    @property
    def status_text(self) -> str:
        return self._status.text()

    @property
    def tau_boxes(self) -> list[QDoubleSpinBox]:
        return self._tau_boxes

    @property
    def estimated_tau(self):
        """Last finite-difference torque estimate (kinematic mode) or None."""
        if self._engine is None or not self._estimate.isChecked():
            return None
        return self._est_tau

    def button_enabled(self) -> dict[str, bool]:
        return dict(start=self._btn_start.isEnabled(), pause=self._btn_pause.isEnabled(),
                    resume=self._btn_resume.isEnabled(), stop=self._btn_stop.isEnabled())

    # ---------------------------------------------------------------- model
    def set_model(self, model: Any, settings: Any, reload_settings: bool = True) -> None:
        """Robot opened / replaced / edited / closed (None): new engine, STOPPED."""
        self._timer.stop()
        self._discard_player()
        old = self._engine
        if old is not None:
            try:
                old.clock.on_state_changed.remove(self._on_clock)
            except ValueError:
                pass
        self._engine = None
        self._recorder = None
        self._model, self._settings = model, settings
        self._samples.clear()
        self._est_tau = None
        self._est_label.setText("")
        err = None
        if model is not None and settings is not None and model.dof() > 0:
            try:
                self._engine = SimEngine(model, settings)
                self._recorder = Recorder(self._engine, model)
            except Exception as exc:  # noqa: BLE001  (compile / dyn build failure)
                err = str(exc)
        elif model is not None:
            err = "no DOF"
        if self._engine is not None:
            self._base_friction = self._engine.friction
            if reload_settings:
                self._set_quiet(self._integ_combo, "setCurrentText", settings.integrator)
                self._set_quiet(self._dt, "setValue", float(settings.physics_timestep))
                self._set_quiet(self._scale, "setValue", 1.0)
                self._set_quiet(self._gravity, "setChecked", True)
                self._set_quiet(self._friction, "setChecked", True)
            self._apply_widgets()
            self._engine.clock.on_state_changed.append(self._on_clock)
            self._engine.reset(current_q(model))
        self._rebuild_tau_boxes()
        if model is not None and model.dof() > 0:
            self._timeline.set_model(model.dof(), self._dof_names or None, dof_limits(model))
        else:
            self._timeline.set_model(0, None, None)
        if self._engine is None:
            self._status.setText(f"No engine: {err}" if err else "No robot")
        self._update_controls()
        self.model_replaced.emit()

    @staticmethod
    def _set_quiet(widget, method, value) -> None:
        widget.blockSignals(True)
        try:
            getattr(widget, method)(value)
        finally:
            widget.blockSignals(False)

    def _rebuild_tau_boxes(self) -> None:
        while self._tau_form.rowCount():
            self._tau_form.removeRow(0)
        self._tau_boxes = []
        self._dof_names = []
        if self._engine is None:
            return
        names = [f"dof {i}" for i in range(self._engine.n_dof)]
        for j_idx, q_idx in enumerate(self._model.q_index_map()):
            if q_idx >= 0:
                names[q_idx] = self._model.joints[j_idx].name or names[q_idx]
        self._dof_names = list(names)
        for name in names:
            sb = QDoubleSpinBox()
            sb.setDecimals(4)
            sb.setRange(-1e6, 1e6)
            sb.setSingleStep(0.1)
            sb.valueChanged.connect(self._on_tau_changed)
            self._tau_form.addRow(name, sb)
            self._tau_boxes.append(sb)

    # ------------------------------------------------- widget -> engine
    def _apply_widgets(self) -> None:
        e = self._engine
        if e is None:
            return
        e.dt = float(self._dt.value())
        e.time_scale = float(self._scale.value())
        e.integrator = make_integrator(self._integ_combo.currentText(),
                                       **e._rk45_kwargs(self._settings))
        self._apply_env()

    def _apply_env(self) -> None:
        e = self._engine
        if e is None:
            return
        g = tuple(float(v) for v in self._settings.gravity)
        e.gravity = g if self._gravity.isChecked() else (0.0, 0.0, 0.0)
        e.friction = self._base_friction if self._friction.isChecked() else None

    def _on_integrator_changed(self, name: str) -> None:
        if self._engine is not None:
            self._engine.integrator = make_integrator(
                name, **self._engine._rk45_kwargs(self._settings))

    def _on_dt_changed(self, v: float) -> None:
        if self._engine is not None:
            self._engine.dt = float(v)

    def _on_scale_changed(self, v: float) -> None:
        if self._engine is not None:
            self._engine.time_scale = float(v)

    def _on_env_changed(self, _checked: bool) -> None:
        if self._engine is None:
            return
        self._apply_env()
        if self._engine.clock.state == STOPPED:   # refresh cached qdd / tau_inv / energy
            s = self._engine.state
            self._engine.reset(s.q, s.qd)
            self._emit_state()

    def _on_tau_changed(self, _v: float) -> None:
        if self._engine is not None:
            self._engine.set_tau([b.value() for b in self._tau_boxes])

    def _on_mode_changed(self, mode: str) -> None:
        self._mode = mode
        self._discard_player()
        if self._engine is not None:
            sim_mode.switch_mode(self._engine, mode)
            self._emit_state()
        self._update_controls()

    def set_mode(self, mode: str) -> None:
        sim_mode.check_mode(mode)
        self._mode_combo.setCurrentText(mode)

    # ------------------------------------------------------------- commands
    def start(self) -> None:
        if (self._engine is not None and self._mode == sim_mode.DYNAMIC
                and self._engine.clock.state == STOPPED):
            self._engine.start()

    def pause(self) -> None:
        if self._engine is not None and self._engine.clock.state == RUNNING:
            self._engine.pause()

    def resume(self) -> None:
        if self._engine is not None and self._engine.clock.state == PAUSED:
            self._engine.error = None
            self._engine.resume()

    def stop(self) -> None:
        if self._engine is not None and self._engine.clock.state in (RUNNING, PAUSED):
            self._engine.stop()
            self._emit_state()

    def shutdown(self) -> None:
        self._timer.stop()

    # ---------------------------------------------------- joint panel edits
    def notify_q_edited(self) -> None:
        """The user moved a joint slider (joint panel q_changed)."""
        e = self._engine
        if e is None or self._model is None:
            return
        self._discard_player()
        if self._mode == sim_mode.KINEMATIC:
            q = current_q(self._model)
            sim_mode.set_kinematic_q(e, q)
            if self._estimate.isChecked():
                self._samples.append((time.perf_counter(), np.asarray(q, float)))
                del self._samples[:-3]
                tau = sim_mode.finite_difference_torque(e, self._samples)
                if tau is not None:
                    self._est_tau = tau
                    e.state.tau_inv = tau
                    self._est_label.setText("tau_est: " + ", ".join(f"{v:.3f}" for v in tau))
            self._refresh_status()
        elif e.clock.state == STOPPED:
            e.reset(current_q(self._model))   # new initial state
            self._refresh_status()
        # DYNAMIC and RUNNING / PAUSED: the simulation owns q; the next tick overwrites it.

    # --------------------------------------------------------- clock / timer
    def _on_clock(self, new: str, _old: str) -> None:
        if new == STOPPED:
            self._discard_player()
        self._update_controls()

    # ------------------------------------------------------ trajectory (R6.4)
    def _discard_player(self) -> None:
        self._bg_play = False
        if self._player is not None:
            self._player.detach()
            self._player = None

    def _on_traj_changed(self, traj) -> None:
        self._discard_player()
        self.trajectory_changed.emit(traj)
        self._update_controls()

    def _ensure_player(self) -> TrajectoryPlayer | None:
        e, traj = self._engine, self._timeline.playable
        if e is None or traj is None or traj.n_dof != e.n_dof:
            return None
        if self._player is None or self._player.traj is not traj or self._player.mode != self._mode:
            self._discard_player()
            kp, kd = resolve_gains(self._settings, e.n_dof)
            self._player = TrajectoryPlayer(e, traj, self._mode, kp, kd,
                                            self._timeline.speed, self._timeline.loop)
            self._player.collision_check = self._collision_check
        return self._player

    def set_pose_check(self, fn) -> None:
        """U3.4: ``fn(q) -> None | text``; Teach Add refuses a colliding pose."""
        self._timeline.set_pose_check(fn)

    def set_collision_check(self, fn) -> None:
        """U3.4: ``fn(q) -> (contact_keys, text)``; playback stops at the last free sample."""
        self._collision_check = fn
        if self._player is not None:
            self._player.collision_check = fn

    def play_trajectory(self) -> None:
        e = self._engine
        if e is None or self._timeline.playable is None:
            return
        if self._player is None and e.clock.state != STOPPED:
            e.stop()   # free-running dynamics: back to the initial state first
        bg = self._bg_play                   # creating the player discards the old one and its flag
        pl = self._ensure_player()
        self._bg_play = bg and pl is not None
        if pl is None:
            return
        pl.play()
        self._last_tick = time.perf_counter()
        self._update_controls()
        self._emit_state()

    def pause_trajectory(self) -> None:
        if self._player is not None:
            self._player.pause()
            self._update_controls()

    def stop_trajectory(self) -> None:
        pl = self._player
        if pl is not None:
            pl.stop()
            self._discard_player()
            self._timeline.set_position(0.0)
            self._update_controls()
            self._emit_state()
            self.position_changed.emit(0.0)

    @property
    def is_playing(self) -> bool:
        """A trajectory is being played (clock running with a player)."""
        e = self._engine
        return e is not None and self._player is not None and e.clock.state == RUNNING

    @property
    def playback_time(self) -> float:
        return float(self._player.t) if self._player is not None else 0.0

    def step_trajectory(self, dt: float) -> None:
        """U6.5: move the playback position by ``dt`` seconds (frame step); pauses a running play."""
        traj = self._timeline.playable
        if traj is None:
            return
        if self.is_playing:
            self.pause_trajectory()
        self.seek_trajectory(min(max(self.playback_time + float(dt), 0.0), traj.duration))

    def set_loop(self, on: bool) -> None:
        self._timeline._loop.setChecked(bool(on))

    def seek_trajectory(self, t: float) -> None:
        pl = self._ensure_player()
        if pl is None:
            return
        pl.seek(t)
        self._update_controls()
        self._emit_state()

    def _on_traj_speed(self, v: float) -> None:
        if self._player is not None:
            self._player.speed = v

    def _on_traj_loop(self, on: bool) -> None:
        if self._player is not None:
            self._player.loop = on

    def _update_controls(self) -> None:
        e = self._engine
        st = e.clock.state if e is not None else STOPPED
        en = sim_mode.button_states(self._mode, st, e is not None)
        self._btn_start.setEnabled(en["start"])
        self._btn_pause.setEnabled(en["pause"])
        self._btn_resume.setEnabled(en["resume"])
        self._btn_stop.setEnabled(en["stop"])
        has = e is not None
        pl = self._player
        can_play = has and self._timeline.playable is not None and st != RUNNING
        self._timeline.set_transport(
            can_play, has and pl is not None and st == RUNNING,
            has and (pl is not None or st in (RUNNING, PAUSED)))
        self._tau_group.setEnabled(has and self._mode == sim_mode.DYNAMIC)
        self._estimate.setEnabled(has and self._mode == sim_mode.KINEMATIC)
        for w in (self._mode_combo, self._integ_combo, self._dt, self._scale,
                  self._gravity, self._friction):
            w.setEnabled(has)
        if has and st == RUNNING and self._shown():
            if not self._timer.isActive():
                self._last_tick = time.perf_counter()
                self._timer.start(self._interval_ms())
        else:
            self._timer.stop()
        self._refresh_status()
        self.transport_changed.emit()

    def _interval_ms(self) -> int:
        fps = float(getattr(self._settings, "render_framerate", DEFAULT_FPS) or DEFAULT_FPS)
        return max(1, int(round(1000.0 / max(1.0, fps))))

    def _shown(self) -> bool:
        return self.isVisible() or self._bg_play

    def set_background_play(self, on: bool) -> None:
        """U4a: keep the playback clock running while the dock is hidden (Play to target from the
        Target dock). Cleared when the player is discarded (new trajectory, stop)."""
        self._bg_play = bool(on)
        self._update_controls()

    def on_visibility_changed(self, visible: bool) -> None:
        """Dock visibility (a hidden tabbed dock does not always deliver hideEvent)."""
        if visible:
            self._update_controls()
        elif not self._bg_play:
            self._timer.stop()

    def showEvent(self, e) -> None:
        super().showEvent(e)
        self._update_controls()

    def hideEvent(self, e) -> None:
        if not self._bg_play:
            self._timer.stop()
        super().hideEvent(e)

    def _on_tick(self) -> None:
        eng = self._engine
        if eng is None or eng.clock.state != RUNNING or not self._shown():
            self._timer.stop()
            return
        now = time.perf_counter()
        wall_dt = min(now - self._last_tick, MAX_WALL_DT)
        self._last_tick = now
        reached = False
        if self._player is not None:
            reached = bool(self._player.advance(wall_dt))
        else:
            eng.advance(wall_dt)
        self._emit_state()
        if reached:
            self.playback_finished.emit()
        self._report_collision_stop()
        if eng.error:
            self.log_requested.emit(f"[WARNING] Simulation paused: {eng.error}")

    def _report_collision_stop(self) -> None:
        pl = self._player
        stop = pl.pop_collision_stop() if pl is not None else None
        if stop is not None:
            self._update_controls()
            self.collision_stopped.emit(float(stop[0]), stop[1])
            self.log_requested.emit(f"[WARNING] Playback stopped: collision at t = {stop[0]:.3f} s "
                                    f"({stop[1]})")

    def _emit_state(self) -> None:
        if self._engine is not None:
            self.state_updated.emit(self._engine.state)
            if self._player is not None:
                self._timeline.set_position(self._player.t)
                self.position_changed.emit(float(self._player.t))
        self._refresh_status()

    def _refresh_status(self) -> None:
        e = self._engine
        if e is None:
            return
        s = e.state
        txt = f"{e.clock.state}  t={s.t:.3f} s  steps={e.steps}  lag={e.lag:.3f} s"
        if self._player is not None:
            txt += f"  traj t={self._player.t:.3f}/{self._player.duration:.3f} s"
            if self._player.mode == sim_mode.DYNAMIC:
                err = self._player.tracking_error()
                txt += f"  track err rms={err['rms']:.2e} max={err['max']:.2e} rad"
        if self._recorder is not None and len(self._recorder):
            txt += "  " + self._recorder.status()
        if e.error:
            txt += f"  ERROR: {e.error}"
        self._status.setText(txt)
