"""
tests/python/test_viewport_smoke.py
R0.7: Verifies PyVista offscreen rendering and Qt viewport widget.
"""

import os
import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import pyvista as pv
pv.global_theme.multi_rendering_splitting_position = 0.0


def test_pyvista_import():
    """PyVista and pyvistaqt import without error."""
    import pyvista as pv
    import pyvistaqt
    assert pv is not None
    assert pyvistaqt is not None


def test_pyvista_offscreen_sphere():
    """PyVista renders a sphere offscreen without crash."""
    plotter = pv.Plotter(off_screen=True, window_size=[200, 200])
    sphere = pv.Sphere()
    plotter.add_mesh(sphere, color="lightblue")
    plotter.show(auto_close=False)

    # Verify the render window exists and has correct size
    assert plotter.window_size[0] == 200
    assert plotter.window_size[1] == 200

    plotter.close()


def test_pyvista_sphere_mesh_properties():
    """PyVista sphere mesh has correct geometry properties."""
    sphere = pv.Sphere(radius=2.0, center=(1.0, 2.0, 3.0))

    # Sphere must have points and cells
    assert sphere.n_points > 0
    assert sphere.n_cells > 0

    # Bounds check: sphere of radius 2 centered at (1,2,3)
    # x: [-1, 3], y: [0, 4], z: [1, 5]
    bounds = sphere.bounds  # (xmin, xmax, ymin, ymax, zmin, zmax)
    assert bounds[0] < 0.0   # xmin < -1 + epsilon
    assert bounds[1] > 2.0   # xmax > 1 + epsilon
    assert sphere.center[0] == pytest.approx(1.0, abs=0.1)
    assert sphere.center[1] == pytest.approx(2.0, abs=0.1)
    assert sphere.center[2] == pytest.approx(3.0, abs=0.1)


def test_viewport_widget_creation(qapp):
    """ViewportWidget creates inside a Qt application without crash."""
    from app.viewport import ViewportWidget

    widget = ViewportWidget()
    assert widget is not None
    assert widget.plotter is not None
    widget.close()


def test_viewport_widget_add_sphere(qapp):
    """ViewportWidget adds a sphere mesh without crash."""
    from app.viewport import ViewportWidget

    widget = ViewportWidget()
    widget.add_sphere(center=(0, 0, 0), radius=1.0, color="lightblue")

    # Verify plotter has at least one actor after adding sphere
    actor_count = len(widget.plotter.actors)
    assert actor_count >= 1

    widget.close()
