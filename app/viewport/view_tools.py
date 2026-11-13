"""U5.8: view buttons over the 3D view (Front, Top, Side, Iso, Fit) and link picking (hover name, click).

``ViewButtons`` is a small overlay widget. ``LinkPicker`` listens to the VTK interactor: hovering a
link shows its name (tooltip) and tints it, a click without dragging selects it (``link_clicked``).
"""
from __future__ import annotations

from typing import Any, Callable

from PySide6.QtCore import QEvent, QObject, Qt, Signal
from PySide6.QtGui import QCursor
from PySide6.QtWidgets import QHBoxLayout, QPushButton, QToolTip, QWidget

VIEWS = ("Front", "Top", "Side", "Iso", "Fit")
CLICK_SLOP_PX = 4
HOVER_TINT = (0.31, 0.55, 1.0)

BUTTON_STYLE = """
QWidget#view_buttons { background: rgba(32, 32, 40, 200); border: 1px solid #3a3a55; border-radius: 6px; }
QWidget#view_buttons QPushButton { color: #e8e8f0; background: transparent; border: none;
                                   padding: 4px 10px; min-height: 22px; font-size: 9.5pt; }
QWidget#view_buttons QPushButton:hover { background: #3d6fcc; color: #ffffff; }
QWidget#view_buttons QPushButton:focus { border: 2px solid #ffffff; }
"""


def apply_view(plotter: Any, name: str) -> None:
    """Move the camera to a named view and fit the scene."""
    if name == "Front":
        plotter.view_xz()
    elif name == "Top":
        plotter.view_xy()
    elif name == "Side":
        plotter.view_yz()
    elif name == "Iso":
        plotter.view_isometric()
    plotter.reset_camera()
    plotter.render()


class ViewButtons(QWidget):
    view_requested = Signal(str)

    def __init__(self, parent: QWidget):
        super().__init__(parent)
        self.setObjectName("view_buttons")
        self.setAttribute(Qt.WidgetAttribute.WA_StyledBackground, True)
        self.setStyleSheet(BUTTON_STYLE)
        lay = QHBoxLayout(self)
        lay.setContentsMargins(4, 2, 4, 2)
        lay.setSpacing(0)
        self.buttons: dict[str, QPushButton] = {}
        tips = {"Front": "Look at the robot from the front", "Top": "Look from above",
                "Side": "Look from the side", "Iso": "Isometric view",
                "Fit": "Fit the whole scene in the view"}
        for name in VIEWS:
            b = QPushButton(name)
            b.setToolTip(tips[name])
            b.setAccessibleName(f"{name} view")
            b.clicked.connect(lambda _c=False, n=name: self.view_requested.emit(n))
            lay.addWidget(b)
            self.buttons[name] = b
        self.adjustSize()
        parent.installEventFilter(self)
        self._place()

    def _place(self) -> None:
        p = self.parentWidget()
        if p is not None:
            self.adjustSize()
            self.move(max(0, p.width() - self.width() - 10), 10)
            self.raise_()

    def eventFilter(self, obj: Any, ev: Any) -> bool:
        if ev.type() == QEvent.Type.Resize:
            self._place()
        return False


class LinkPicker(QObject):
    """Hover and click on the robot meshes (visual actors of ``RobotView``)."""

    link_clicked = Signal(int)          # link index
    link_hovered = Signal(int)          # link index, -1 = none

    def __init__(self, viewport: Any, get_model: Callable[[], Any],
                 enabled_fn: Callable[[], bool] | None = None, parent: QObject | None = None):
        super().__init__(parent)
        self._viewport = viewport
        self._get_model = get_model
        self._enabled_fn = enabled_fn
        self._press: tuple[int, int] | None = None
        self._hover = -1
        self._tinted: list = []
        self._picker: Any = None
        self._attached = False
        self.attach()

    # ------------------------------------------------------------------ setup
    def attach(self) -> None:
        if self._attached:
            return
        try:
            import vtk
            self._picker = vtk.vtkPropPicker()
            iren = self._viewport.plotter.iren
            iren.add_observer("MouseMoveEvent", self._on_move)
            iren.add_observer("LeftButtonPressEvent", self._on_press)
            iren.add_observer("LeftButtonReleaseEvent", self._on_release)
            self._attached = True
        except Exception:
            self._attached = False

    @property
    def hovered_link(self) -> int:
        return self._hover

    def _enabled(self) -> bool:
        return self._enabled_fn is None or bool(self._enabled_fn())

    # ------------------------------------------------------------------ picking
    def link_at(self, x: int, y: int) -> int:
        """Link index under the VTK display position (x, y), -1 when none."""
        rv = self._viewport.robot_view
        try:
            plotter = self._viewport.plotter
            self._picker.Pick(x, y, 0, plotter.renderer)
            actor = self._picker.GetActor()
        except Exception:
            return -1
        if actor is None:
            return -1
        for a, li in zip(rv._visual_actors, rv._visual_links):
            if a is actor:
                return int(li)
        return -1

    def _event_pos(self) -> tuple[int, int]:
        x, y = self._viewport.plotter.iren.get_event_position()
        return int(x), int(y)

    def _on_move(self, *_a) -> None:
        if self._press is not None or not self._enabled():
            return
        x, y = self._event_pos()
        self.set_hover(self.link_at(x, y))

    def _on_press(self, *_a) -> None:
        self._press = self._event_pos()

    def _on_release(self, *_a) -> None:
        start, self._press = self._press, None
        if start is None or not self._enabled():
            return
        x, y = self._event_pos()
        if abs(x - start[0]) > CLICK_SLOP_PX or abs(y - start[1]) > CLICK_SLOP_PX:
            return                                  # a drag (orbit, handle), not a click
        li = self.link_at(x, y)
        if li >= 0:
            self.link_clicked.emit(li)

    # ------------------------------------------------------------------ hover look
    def set_hover(self, li: int) -> None:
        if li == self._hover:
            return
        self._clear_tint()
        self._hover = li
        rv = self._viewport.robot_view
        if li >= 0:
            for a, k in zip(rv._visual_actors, rv._visual_links):
                if k == li:
                    try:
                        a.prop.color = HOVER_TINT
                        self._tinted.append(a)
                    except Exception:
                        pass
            model = self._get_model()
            name = ""
            try:
                name = model.links[li].name
            except Exception:
                name = f"link {li}"
            QToolTip.showText(QCursor.pos(), name, self._viewport)
        else:
            QToolTip.hideText()
        self.link_hovered.emit(li)
        try:
            self._viewport.plotter.render()
        except Exception:
            pass

    def _clear_tint(self) -> None:
        if self._tinted:
            self._tinted = []
            try:
                self._viewport.robot_view._apply_collision_highlight()    # restores every colour
            except Exception:
                pass
