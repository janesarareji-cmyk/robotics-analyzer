"""
app/viewport.py
Robotics Analyzer — 3D viewport widget.
R1.7: PyVista/VTK viewport docked into the MainWindow central area.
"""

from PySide6.QtWidgets import QWidget, QVBoxLayout


class ViewportWidget(QWidget):
    """
    3D viewport backed by pyvistaqt.QtInteractor.
    VTK/PyVista are imported lazily inside __init__ so the app module
    can be imported in headless test environments without loading VTK.
    """

    def __init__(self, parent=None, bg_color: str = "#1a1a2e"):
        super().__init__(parent)
        # Lazy import — VTK must NOT load at module import time
        from pyvistaqt import QtInteractor  # noqa: PLC0415
        from app.viewport.robot_view import RobotView  # noqa: PLC0415
        self._plotter = QtInteractor(parent=self)
        self._robot_view = RobotView(self._plotter)
        if bg_color:
            self._plotter.set_background(bg_color)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.addWidget(self._plotter)
        self.setLayout(layout)

    @property
    def plotter(self):
        """Return the underlying PyVista plotter."""
        return self._plotter

    @property
    def robot_view(self):
        """Return the RobotView manager."""
        return self._robot_view

    def add_mesh(self, mesh, **kwargs):
        """Add a PyVista mesh to the viewport."""
        self._plotter.add_mesh(mesh, **kwargs)
        self._plotter.reset_camera()

    def add_sphere(self, center=(0.0, 0.0, 0.0), radius=1.0, color="lightblue"):
        """Add a sphere mesh to the viewport."""
        import pyvista as pv
        sphere = pv.Sphere(center=center, radius=radius)
        self.add_mesh(sphere, color=color, show_edges=True)

    def clear(self):
        """Remove all actors from the viewport."""
        self._plotter.clear()

    def reset_camera(self):
        """Reset the camera to fit all actors."""
        self._plotter.reset_camera()

    def set_background(self, color: str):
        """Set the viewport background color."""
        self._plotter.set_background(color)

    def close(self):
        """Clean up the plotter before closing."""
        if hasattr(self, "_plotter"):
            self._plotter.close()
        super().close()
