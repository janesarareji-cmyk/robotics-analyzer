"""U5.7: Cartesian jog buttons (X/Y/Z, Rx/Ry/Rz), step sizes, base or tool frame, hold to repeat."""
from __future__ import annotations

from PySide6.QtCore import Qt, Signal
from PySide6.QtWidgets import QComboBox, QGridLayout, QGroupBox, QHBoxLayout, QLabel, QPushButton, QVBoxLayout

from app.cartesian_jog import ANG_STEPS_DEG, LIN_STEPS_MM, CartesianJog

REPEAT_DELAY_MS = 350
REPEAT_INTERVAL_MS = 90

_NAMES = {"x": "X", "y": "Y", "z": "Z", "rx": "Rx", "ry": "Ry", "rz": "Rz"}


class JogPanel(QGroupBox):
    """Group box with the jog buttons. Pressing and holding a button repeats the step."""

    status = Signal(str)

    def __init__(self, jog: CartesianJog, parent=None):
        super().__init__("Move the tool (Cartesian jog)", parent)
        self.jog = jog
        self.buttons: dict[tuple[str, int], QPushButton] = {}
        lay = QVBoxLayout(self)
        lay.setContentsMargins(6, 8, 6, 6)
        lay.setSpacing(6)

        opts = QGridLayout()
        opts.setHorizontalSpacing(6)
        self._lin = QComboBox()
        for v in LIN_STEPS_MM:
            self._lin.addItem(f"{v:g} mm", v)
        self._lin.setCurrentIndex(LIN_STEPS_MM.index(10.0))
        self._lin.setToolTip("How far one press moves the tool")
        self._lin.setAccessibleName("Move step")
        self._ang = QComboBox()
        for v in ANG_STEPS_DEG:
            self._ang.addItem(f"{v:g} deg", v)
        self._ang.setCurrentIndex(ANG_STEPS_DEG.index(5.0))
        self._ang.setToolTip("How far one press turns the tool")
        self._ang.setAccessibleName("Turn step")
        self._frame = QComboBox()
        self._frame.addItem("Base frame", "base")
        self._frame.addItem("Tool frame", "tool")
        self._frame.setToolTip("Move along the axes of the robot base, or along the axes of the tool")
        self._frame.setAccessibleName("Jog frame")
        opts.addWidget(QLabel("Move"), 0, 0)
        opts.addWidget(self._lin, 0, 1)
        opts.addWidget(QLabel("Turn"), 0, 2)
        opts.addWidget(self._ang, 0, 3)
        opts.addWidget(QLabel("In"), 1, 0)
        opts.addWidget(self._frame, 1, 1, 1, 3)
        lay.addLayout(opts)

        grid = QGridLayout()
        grid.setSpacing(4)
        for r, (lin, ang) in enumerate((("x", "rx"), ("y", "ry"), ("z", "rz"))):
            for c, (axis, sign) in enumerate(((lin, -1), (lin, +1), (ang, -1), (ang, +1))):
                sym = "-" if sign < 0 else "+"
                b = QPushButton(f"{_NAMES[axis]}{sym}")
                b.setMinimumHeight(28)
                b.setAutoRepeat(True)
                b.setAutoRepeatDelay(REPEAT_DELAY_MS)
                b.setAutoRepeatInterval(REPEAT_INTERVAL_MS)
                kind = "Move" if axis in ("x", "y", "z") else "Turn"
                direction = "negative" if sign < 0 else "positive"
                b.setToolTip(f"{kind} the tool {direction} {_NAMES[axis]} (hold to repeat)")
                b.setAccessibleName(f"{kind} {_NAMES[axis]} {direction}")
                b.clicked.connect(lambda _c=False, a=axis, s=sign: self.step(a, s))
                grid.addWidget(b, r, c)
                self.buttons[(axis, sign)] = b
        lay.addLayout(grid)
        self._msg = QLabel("")
        self._msg.setWordWrap(True)
        self._msg.setStyleSheet("color: #ffb86b;")
        lay.addWidget(self._msg)
        self._lin.currentIndexChanged.connect(lambda _i: self._sync())
        self._ang.currentIndexChanged.connect(lambda _i: self._sync())
        self._frame.currentIndexChanged.connect(lambda _i: self._sync())
        jog.failed.connect(self._show)
        jog.blocked.connect(self._show)
        self._sync()

    def _sync(self) -> None:
        self.jog.lin_mm = float(self._lin.currentData())
        self.jog.ang_deg = float(self._ang.currentData())
        self.jog.set_frame(str(self._frame.currentData()))

    def _show(self, text: str) -> None:
        self._msg.setText(text)
        self.status.emit(text)

    def step(self, axis: str, sign: int) -> bool:
        self._sync()
        self._msg.setText("")
        return self.jog.jog(axis, sign)

    def set_steps(self, lin_mm: float | None = None, ang_deg: float | None = None, frame: str | None = None) -> None:
        if lin_mm is not None:
            self._lin.setCurrentIndex(self._lin.findData(float(lin_mm)))
        if ang_deg is not None:
            self._ang.setCurrentIndex(self._ang.findData(float(ang_deg)))
        if frame is not None:
            self._frame.setCurrentIndex(self._frame.findData(frame))
        self._sync()

    @property
    def message(self) -> str:
        return self._msg.text()
