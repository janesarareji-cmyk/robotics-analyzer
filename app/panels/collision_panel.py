"""
app/panels/collision_panel.py
R7.6: "Collision" dock. Edits the project's collision settings (``project.collision``, saved in
``collision.json``): environment shapes (box, sphere, cylinder; pose + size), the allowed-collision
matrix edit list, and the "disable pairs always colliding at the zero pose" helper.

The panel changes ``project.collision`` in place (a ``CollisionConfig`` is created on the first
edit) and emits ``changed``; the Planning panel reads the project's config when it plans.
"""

from __future__ import annotations

from typing import Any

from PySide6.QtCore import Signal
from PySide6.QtWidgets import (
    QAbstractItemView, QCheckBox, QComboBox, QDoubleSpinBox, QGridLayout, QHBoxLayout, QLabel,
    QLineEdit, QPushButton, QSpinBox, QTableWidget, QTableWidgetItem, QVBoxLayout, QWidget,
)

from app.planning.collision import (
    DEFAULT_FIT_MODE, FIT_MODES, CollisionConfig, CollisionWorld, EnvShape, shapes_from_project,
)

_KINDS = ("box", "sphere", "cylinder")
_FIT_LABELS = {"hull": "Convex hull", "obb": "Box (OBB)", "sphere": "Sphere", "capsule": "Capsule"}


def _fmt(x: float) -> str:
    return f"{x:.4g}"


def _params_text(kind: str, params: dict) -> str:
    if kind == "box":
        return "size " + " x ".join(_fmt(v) for v in params.get("size", []))
    if kind == "sphere":
        return f"r {_fmt(params.get('radius', 0.0))}"
    return f"r {_fmt(params.get('radius', 0.0))}, length {_fmt(params.get('length', 0.0))}"


class CollisionPanel(QWidget):
    """Dock content for environment shapes and the allowed-collision matrix."""

    log_requested = Signal(str)
    changed = Signal()          # project.collision edited (project becomes dirty)
    auto_acm_requested = Signal()   # U3.2: recompute the automatic matrix
    shape_selected = Signal(int)    # U3.6: index of the selected environment object, -1 = none

    def __init__(self, parent=None):
        super().__init__(parent)
        self._project: Any = None
        self._n_links = 0
        self._model: Any = None
        self._loading = False

        layout = QVBoxLayout(self)
        layout.setContentsMargins(6, 6, 6, 6)
        layout.setSpacing(4)

        frow = QHBoxLayout()
        frow.addWidget(QLabel("Mesh fit"))
        self._fit = QComboBox()
        for mode in FIT_MODES:
            self._fit.addItem(_FIT_LABELS[mode], mode)
        self._fit.currentIndexChanged.connect(self._on_fit_changed)
        frow.addWidget(self._fit, 1)
        layout.addLayout(frow)
        self._floor = QCheckBox("Floor collision")
        self._floor.setToolTip("The floor is a plane 1 mm below the robot base. Links fixed to the "
                               "base are not checked against it.")
        self._floor.setChecked(True)
        self._floor.toggled.connect(self._on_floor_toggled)
        layout.addWidget(self._floor)

        layout.addWidget(QLabel("Environment shapes"))
        self._env = QTableWidget(0, 5)
        self._env.setHorizontalHeaderLabels(["Name", "Kind", "xyz [m]", "rpy [rad]", "Size"])
        self._env.setSelectionBehavior(QAbstractItemView.SelectionBehavior.SelectRows)
        self._env.setSelectionMode(QAbstractItemView.SelectionMode.SingleSelection)
        self._env.setEditTriggers(QAbstractItemView.EditTrigger.NoEditTriggers)
        self._env.setMinimumHeight(90)
        self._env.itemSelectionChanged.connect(self._on_env_selection)
        layout.addWidget(self._env, 1)

        grid = QGridLayout()
        self._name = QLineEdit()
        self._name.setPlaceholderText("name")
        self._kind = QComboBox()
        self._kind.addItems(_KINDS)
        self._kind.currentTextChanged.connect(self._on_kind)
        grid.addWidget(self._name, 0, 0, 1, 2)
        grid.addWidget(self._kind, 0, 2, 1, 2)
        self._pos = [self._spin(-100.0, 100.0, 0.0) for _ in range(3)]
        self._rot = [self._spin(-7.0, 7.0, 0.0) for _ in range(3)]
        for i, (lab, w) in enumerate(zip(("x", "y", "z"), self._pos)):
            grid.addWidget(QLabel(lab), 1, 2 * i)
            grid.addWidget(w, 1, 2 * i + 1)
        for i, (lab, w) in enumerate(zip(("roll", "pitch", "yaw"), self._rot)):
            grid.addWidget(QLabel(lab), 2, 2 * i)
            grid.addWidget(w, 2, 2 * i + 1)
        self._size = [self._spin(0.001, 100.0, 0.2) for _ in range(3)]
        self._size_labels = [QLabel("") for _ in range(3)]
        for i in range(3):
            grid.addWidget(self._size_labels[i], 3, 2 * i)
            grid.addWidget(self._size[i], 3, 2 * i + 1)
        layout.addLayout(grid)
        row = QHBoxLayout()
        self._btn_add = QPushButton("Add shape")
        self._btn_add.clicked.connect(lambda: self._add_from_form())
        self._btn_remove = QPushButton("Remove selected")
        self._btn_remove.clicked.connect(self._remove_selected_shape)
        self._btn_update = QPushButton("Update selected")
        self._btn_update.setToolTip("Write the name, size, position and rotation above into the "
                                    "selected object")
        self._btn_update.clicked.connect(lambda: self._update_selected_from_form())
        row.addWidget(self._btn_add)
        row.addWidget(self._btn_update)
        row.addWidget(self._btn_remove)
        layout.addLayout(row)

        layout.addWidget(QLabel("Allowed-collision matrix edits"))
        self._acm = QTableWidget(0, 4)
        self._acm.setHorizontalHeaderLabels(["Link A", "Link B", "Check", "Source"])
        self._acm.setSelectionBehavior(QAbstractItemView.SelectionBehavior.SelectRows)
        self._acm.setSelectionMode(QAbstractItemView.SelectionMode.SingleSelection)
        self._acm.setEditTriggers(QAbstractItemView.EditTrigger.NoEditTriggers)
        self._acm.setMinimumHeight(80)
        layout.addWidget(self._acm, 1)
        arow = QHBoxLayout()
        self._a = QSpinBox()
        self._b = QSpinBox()
        self._check = QCheckBox("check this pair")
        self._btn_acm_add = QPushButton("Set edit")
        self._btn_acm_add.clicked.connect(lambda: self.set_acm_edit(
            self._a.value(), self._b.value(), self._check.isChecked()))
        self._btn_acm_rm = QPushButton("Remove selected")
        self._btn_acm_rm.clicked.connect(self._remove_selected_edit)
        for w in (QLabel("A"), self._a, QLabel("B"), self._b, self._check,
                  self._btn_acm_add, self._btn_acm_rm):
            arow.addWidget(w)
        layout.addLayout(arow)
        self._btn_zero = QPushButton("Disable pairs always colliding at zero pose")
        self._btn_zero.clicked.connect(self.disable_zero_pose_pairs)
        layout.addWidget(self._btn_zero)
        self._btn_auto = QPushButton("Recompute automatic matrix")
        self._btn_auto.setToolTip("Sample the joint limits and disable pairs that always or never "
                                  "collide. Your own edits are kept.")
        self._btn_auto.clicked.connect(self.auto_acm_requested.emit)
        layout.addWidget(self._btn_auto)

        self._status = QLabel("")
        self._status.setWordWrap(True)
        layout.addWidget(self._status)
        self._on_kind("box")
        self._refresh()

    @staticmethod
    def _spin(lo: float, hi: float, val: float) -> QDoubleSpinBox:
        s = QDoubleSpinBox()
        s.setRange(lo, hi)
        s.setDecimals(4)
        s.setSingleStep(0.05)
        s.setValue(val)
        return s

    # ------------------------------------------------------------------ facts
    @property
    def status_text(self) -> str:
        return self._status.text()

    @property
    def env_table(self) -> QTableWidget:
        return self._env

    @property
    def acm_table(self) -> QTableWidget:
        return self._acm

    @property
    def config(self) -> CollisionConfig | None:
        return None if self._project is None else getattr(self._project, "collision", None)

    # ------------------------------------------------------------------ setup
    def set_project(self, project: Any) -> None:
        self._project = project
        self._env.clearSelection()
        self._refresh()

    def set_model(self, model: Any) -> None:
        self._model = model
        self._n_links = len(model.links) if model is not None else 0
        top = max(self._n_links - 1, 0)
        self._a.setRange(0, top)
        self._b.setRange(0, top)
        self._refresh()

    def _ensure_config(self) -> CollisionConfig | None:
        if self._project is None:
            self._status.setText("No project open.")
            return None
        if getattr(self._project, "collision", None) is None:
            self._project.collision = CollisionConfig()
        return self._project.collision

    def _edited(self, message: str, keep: int | None = None) -> None:
        self._refresh(keep)
        self._status.setText(message)
        self.log_requested.emit(f"[INFO] Collision: {message}")
        self.changed.emit()

    # ------------------------------------------------------------------ mesh fit
    @property
    def fit_mode(self) -> str:
        return self._fit.currentData()

    def set_fit_mode(self, mode: str) -> bool:
        """Set the mesh fit of the project's collision shapes (hull, obb, sphere, capsule)."""
        if mode not in FIT_MODES:
            self._status.setText(f"Unknown fit mode {mode!r}.")
            return False
        cfg = self._ensure_config()
        if cfg is None:
            return False
        if cfg.fit_mode != mode:
            cfg.fit_mode = mode
            self._edited(f"Mesh fit: {_FIT_LABELS[mode]}.")
        else:
            self._refresh()
        return True

    def _on_fit_changed(self, _i: int) -> None:
        if self._loading or self._project is None:
            return
        self.set_fit_mode(self._fit.currentData())

    @property
    def floor_enabled(self) -> bool:
        return self._floor.isChecked()

    def set_floor(self, on: bool) -> bool:
        """Switch the floor plane on or off (saved in ``collision.json``)."""
        cfg = self._ensure_config()
        if cfg is None:
            return False
        if cfg.floor != bool(on):
            cfg.floor = bool(on)
            self._edited(f"Floor collision {'on' if on else 'off'}.")
        else:
            self._refresh()
        return True

    def _on_floor_toggled(self, on: bool) -> None:
        if self._loading or self._project is None:
            return
        self.set_floor(on)

    # ------------------------------------------------------------------ environment
    def _on_kind(self, kind: str) -> None:
        labels = {"box": ("size x", "size y", "size z"), "sphere": ("radius", "", ""),
                  "cylinder": ("radius", "length", "")}[kind]
        for lab, w, text in zip(self._size_labels, self._size, labels):
            lab.setText(text)
            lab.setVisible(bool(text))
            w.setVisible(bool(text))

    def add_shape(self, kind: str, params: dict, xyz=(0.0, 0.0, 0.0), rpy=(0.0, 0.0, 0.0),
                  name: str = "") -> bool:
        """Add an environment shape. False (status text) on a bad shape or no project."""
        cfg = self._ensure_config()
        if cfg is None:
            return False
        sizes = []
        for v in params.values():
            sizes.extend(v if isinstance(v, (list, tuple)) else [v])
        try:
            bad = not sizes or any(float(x) <= 0.0 for x in sizes)
        except (TypeError, ValueError):
            bad = True
        if bad:
            self._status.setText("Bad shape: every size must be positive.")
            return False
        try:
            shape = EnvShape(kind, dict(params), [float(v) for v in xyz],
                             [float(v) for v in rpy], name or f"{kind} {len(cfg.environment) + 1}")
        except (ValueError, KeyError, TypeError) as exc:
            self._status.setText(f"Bad shape: {exc}")
            return False
        cfg.environment.append(shape)
        self._edited(f"Added {shape.kind} '{shape.name}'.")
        return True

    def _add_from_form(self) -> bool:
        kind = self._kind.currentText()
        v = [w.value() for w in self._size]
        params = {"box": {"size": v[:3]}, "sphere": {"radius": v[0]},
                  "cylinder": {"radius": v[0], "length": v[1]}}[kind]
        return self.add_shape(kind, params, [w.value() for w in self._pos],
                              [w.value() for w in self._rot], self._name.text().strip())

    def add_default_object(self, kind: str) -> bool:
        """U3.6 toolbar: a 0.1 m object on the floor, 0.4 m in front of the robot, selected."""
        from app.scene_objects import default_object

        if kind not in _KINDS:
            return False
        params, xyz = default_object(kind, self._model)
        if not self.add_shape(kind, params, xyz):
            return False
        self.select_shape(len(self.config.environment) - 1)
        return True

    def refresh(self) -> None:
        """Redraw the lists after the project's collision data changed elsewhere (grasp)."""
        self._refresh()

    @property
    def selected_shape(self) -> int:
        rows = self._env.selectionModel().selectedRows()
        return rows[0].row() if rows else -1

    def select_shape(self, index: int) -> None:
        """Select row ``index`` of the object list (-1 clears)."""
        if 0 <= index < self._env.rowCount():
            self._env.selectRow(index)
        else:
            self._env.clearSelection()

    def _on_env_selection(self) -> None:
        if self._loading:
            return
        i = self.selected_shape
        cfg = self.config
        if cfg is not None and 0 <= i < len(cfg.environment):
            self._load_form(cfg.environment[i])
        self.shape_selected.emit(i)

    def _load_form(self, e: EnvShape) -> None:
        self._loading = True
        self._name.setText(e.name)
        self._kind.setCurrentText(e.kind)
        for w, v in zip(self._pos, e.xyz):
            w.setValue(float(v))
        for w, v in zip(self._rot, e.rpy):
            w.setValue(float(v))
        p = e.params
        vals = {"box": list(p.get("size", [])), "sphere": [p.get("radius", 0.0)],
                "cylinder": [p.get("radius", 0.0), p.get("length", 0.0)]}[e.kind]
        for w, v in zip(self._size, vals):
            w.setValue(float(v))
        self._loading = False

    def update_shape(self, index: int, name: str | None = None, params: dict | None = None,
                     xyz=None, rpy=None) -> bool:
        """Edit object ``index`` (kind stays). False on a bad index or bad size."""
        cfg = self.config
        if cfg is None or not 0 <= index < len(cfg.environment):
            return False
        old = cfg.environment[index]
        try:
            new = EnvShape(old.kind, dict(params if params is not None else old.params),
                           [float(v) for v in (xyz if xyz is not None else old.xyz)],
                           [float(v) for v in (rpy if rpy is not None else old.rpy)],
                           old.name if name is None else name)
            sizes = []
            for v in new.params.values():
                sizes.extend(v if isinstance(v, (list, tuple)) else [v])
            if any(float(x) <= 0.0 for x in sizes):
                raise ValueError("every size must be positive")
        except (ValueError, KeyError, TypeError) as exc:
            self._status.setText(f"Bad shape: {exc}")
            return False
        cfg.environment[index] = new
        self._edited(f"Updated '{new.name}'.", keep=index)
        return True

    def move_shape(self, index: int, xyz) -> bool:
        """Set the position of object ``index`` (drag widget)."""
        return self.update_shape(index, xyz=xyz)

    def _update_selected_from_form(self) -> bool:
        i = self.selected_shape
        cfg = self.config
        if cfg is None or not 0 <= i < len(cfg.environment):
            self._status.setText("Select an object first.")
            return False
        kind = cfg.environment[i].kind
        v = [w.value() for w in self._size]
        params = {"box": {"size": v[:3]}, "sphere": {"radius": v[0]},
                  "cylinder": {"radius": v[0], "length": v[1]}}[kind]
        return self.update_shape(i, self._name.text().strip() or None, params,
                                 [w.value() for w in self._pos], [w.value() for w in self._rot])

    def remove_shape(self, index: int) -> bool:
        cfg = self.config
        if cfg is None or not 0 <= index < len(cfg.environment):
            return False
        gone = cfg.environment.pop(index)
        self._edited(f"Removed '{gone.name}'.")
        return True

    def _remove_selected_shape(self) -> None:
        rows = self._env.selectionModel().selectedRows()
        if rows:
            self.remove_shape(rows[0].row())

    # ------------------------------------------------------------------ ACM
    def set_acm_edit(self, a: int, b: int, check: bool) -> bool:
        cfg = self._ensure_config()
        if cfg is None:
            return False
        for k in (a, b):
            if not 0 <= int(k) < max(self._n_links, 1):
                self._status.setText(f"Link {k} does not exist.")
                return False
        try:
            cfg.set_check(int(a), int(b), bool(check))
        except ValueError as exc:
            self._status.setText(str(exc))
            return False
        self._edited(f"Pair {min(a, b)}-{max(a, b)} {'checked' if check else 'not checked'}.")
        return True

    def remove_acm_edit(self, a: int, b: int) -> bool:
        cfg = self.config
        if cfg is None:
            return False
        key = (min(a, b), max(a, b))
        if key not in cfg.edits:
            return False
        cfg.clear_edit(a, b)
        self._edited(f"Removed edit {key[0]}-{key[1]}.")
        return True

    def _remove_selected_edit(self) -> None:
        rows = self._acm.selectionModel().selectedRows()
        if rows:
            r = rows[0].row()
            self.remove_acm_edit(int(self._acm.item(r, 0).text()), int(self._acm.item(r, 1).text()))

    def disable_zero_pose_pairs(self) -> list[tuple[int, int]]:
        """Stop checking link pairs that collide at the zero pose. Returns the pairs."""
        project = self._project
        model = getattr(project, "robot_model", None)
        if project is None or model is None or len(model.joints) == 0:
            self._status.setText("No robot: nothing to check.")
            return []
        cfg = self._ensure_config()
        try:
            shapes = [s for s in shapes_from_project(project) if s.link >= 0]
            world = CollisionWorld(model, shapes, cfg)
            found = world.disable_always_colliding()
        except Exception as exc:
            self._status.setText(f"Collision check failed: {exc}")
            return []
        if found:
            self._edited("Disabled " + ", ".join(f"{a}-{b}" for a, b in found) + " (collide at zero pose).")
        else:
            self._status.setText("No link pair collides at the zero pose.")
        return found

    # ------------------------------------------------------------------ table refresh
    def _refresh(self, keep: int | None = None) -> None:
        sel = self.selected_shape if keep is None else keep
        cfg = self.config
        self._loading = True
        self._fit.setCurrentIndex(self._fit.findData(cfg.fit_mode if cfg else DEFAULT_FIT_MODE))
        self._floor.setChecked(cfg.floor if cfg else True)
        self._loading = False
        env = list(cfg.environment) if cfg is not None else []
        edits = sorted(cfg.edits.items()) if cfg is not None else []
        rows = [(a, b, chk, "user") for (a, b), chk in edits]
        if cfg is not None:
            rows += [(a, b, False, f"auto ({why})") for (a, b), why in sorted(cfg.auto.items())
                     if (a, b) not in cfg.edits]
        self._loading = True
        self._env.setRowCount(len(env))
        self._loading = False
        for r, e in enumerate(env):
            cells = [e.name, e.kind, "(" + ", ".join(_fmt(v) for v in e.xyz) + ")",
                     "(" + ", ".join(_fmt(v) for v in e.rpy) + ")", _params_text(e.kind, e.params)]
            for c, text in enumerate(cells):
                self._env.setItem(r, c, QTableWidgetItem(text))
        if 0 <= sel < len(env):
            self._loading = True
            self._env.selectRow(sel)
            self._loading = False
        self._acm.setRowCount(len(rows))
        for r, (a, b, chk, src) in enumerate(rows):
            for c, text in enumerate((str(a), str(b), "yes" if chk else "no", src)):
                self._acm.setItem(r, c, QTableWidgetItem(text))
        has = self._project is not None
        for w in (self._btn_add, self._btn_remove, self._btn_acm_add, self._btn_acm_rm,
                  self._btn_zero,
                  self._btn_auto, self._fit, self._floor):
            w.setEnabled(has)
