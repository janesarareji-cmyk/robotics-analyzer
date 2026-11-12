"""Thin plot backend over pyqtgraph (R6.6). Swapping the library touches only this file."""
from __future__ import annotations

import numpy as np
import pyqtgraph as pg
from PySide6.QtCore import QObject, Signal

PALETTE = ("#4fc3f7", "#ffb74d", "#81c784", "#e57373", "#ba68c8", "#fff176",
           "#4db6ac", "#f06292", "#a1887f", "#90a4ae")

pg.setConfigOptions(antialias=False, background="#1e1e1e", foreground="#cccccc")


class PlotView(QObject):
    """2D line plot with legend, units in the axis label and a movable vertical cursor."""

    cursor_moved = Signal(float)   # user drag or ``cursor.setValue`` (not ``set_cursor``)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.widget = pg.PlotWidget()
        self._plot = self.widget.getPlotItem()
        self._plot.showGrid(x=True, y=True, alpha=0.25)
        self._plot.setLabel("bottom", "t", units="s")
        self._legend = self._plot.addLegend(offset=(10, 10))
        self.cursor = pg.InfiniteLine(pos=0.0, angle=90, movable=True,
                                      pen=pg.mkPen("#ffffff", width=1))
        self._plot.addItem(self.cursor)
        self._silent = False
        self._curves = []
        self.cursor.sigPositionChanged.connect(self._on_cursor)

    def _on_cursor(self, _line):
        if not self._silent:
            self.cursor_moved.emit(float(self.cursor.value()))

    # ---------------------------------------------------------------- data
    def set_series(self, series, auto_range=True):
        """Replace the curves by ``series`` (objects with name, unit, t, y)."""
        for c in self._curves:
            self._plot.removeItem(c)
        self._legend.clear()
        self._curves = []
        units = []
        for i, s in enumerate(series):
            curve = self._plot.plot(np.asarray(s.t), np.asarray(s.y),
                                    pen=pg.mkPen(PALETTE[i % len(PALETTE)], width=1.5),
                                    name=f"{s.name} [{s.unit}]")
            self._curves.append(curve)
            if s.unit not in units:
                units.append(s.unit)
        self._plot.setLabel("left", ", ".join(units) if units else "")
        if auto_range:
            self._plot.enableAutoRange()

    def clear(self):
        self.set_series([])

    @property
    def curve_count(self):
        return len(self._curves)

    def point_counts(self):
        return [len(c.xData) if c.xData is not None else 0 for c in self._curves]

    def legend_labels(self):
        return [lbl.text for _s, lbl in self._legend.items]

    def grab_ok(self):
        return not self.widget.grab().isNull()

    # -------------------------------------------------------------- cursor
    def set_cursor(self, t):
        """Move the cursor without emitting ``cursor_moved``."""
        self._silent = True
        try:
            self.cursor.setValue(float(t))
        finally:
            self._silent = False

    def cursor_time(self):
        return float(self.cursor.value())
