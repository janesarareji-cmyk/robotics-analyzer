"""U5.8: view buttons (Front/Top/Side/Iso/Fit), link hover name + outline, click selects in the tree."""
import os

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import numpy as np
import pytest
from PySide6.QtCore import QCoreApplication, Qt
from PySide6.QtTest import QTest

from app.main_window import MainWindow
from app.settings import AppSettings
from app.viewport.view_tools import HOVER_TINT, VIEWS
from tests.python.test_live_blocking import load


@pytest.fixture
def win(qapp, monkeypatch):
    monkeypatch.setattr(AppSettings, "_ORGANIZATION", "RoboticsAnalyzer_test_vt")
    monkeypatch.setattr(AppSettings, "_APPLICATION", "RoboticsAnalyzer_test_vt")
    w = MainWindow()
    w.settings._settings.clear()
    w.resize(1500, 900)
    w.show()
    QCoreApplication.processEvents()
    yield w
    w.project_controller._is_modified = False
    w.sim_dock.hide()
    w.close()


def _dir(win):
    cam = win.viewport.plotter.camera
    d = np.array(cam.position) - np.array(cam.focal_point)
    return d / np.linalg.norm(d)


def test_view_buttons_set_the_camera(win):
    assert not win.view_buttons.isVisible()           # start screen covers an empty view
    load(win, "ur5")
    QCoreApplication.processEvents()
    vb = win.view_buttons
    assert vb.isVisible() and list(vb.buttons) == list(VIEWS)
    assert vb.geometry().right() <= win.viewport.width() and vb.geometry().top() >= 0
    dirs = {}
    for name in ("Front", "Top", "Side", "Iso"):
        QTest.mouseClick(vb.buttons[name], Qt.MouseButton.LeftButton)
        dirs[name] = _dir(win)
    assert abs(abs(dirs["Top"][2]) - 1.0) < 1e-6                 # looks along z
    assert abs(abs(dirs["Front"][1]) - 1.0) < 1e-6               # along y
    assert abs(abs(dirs["Side"][0]) - 1.0) < 1e-6                # along x
    assert np.all(np.abs(dirs["Iso"]) > 0.3)
    # Fit keeps the direction and brings the scene back in view
    win.viewport.plotter.camera.zoom(5.0)
    QTest.mouseClick(vb.buttons["Fit"], Qt.MouseButton.LeftButton)
    assert np.allclose(_dir(win), dirs["Iso"], atol=1e-6)


def test_click_on_link_selects_it_in_the_tree(win, monkeypatch):
    model = load(win, "ur5")
    picker = win.link_picker
    monkeypatch.setattr(picker, "link_at", lambda x, y: 3)
    pos = iter([(100, 100), (101, 100)])
    monkeypatch.setattr(picker, "_event_pos", lambda: next(pos))
    got = []
    picker.link_clicked.connect(got.append)
    picker._on_press()
    picker._on_release()
    assert got == [3]
    item = win.model_panel._tree.currentItem()
    assert item is not None and win.props_panel is not None
    assert win.model_panel._link_items[3] is item


def test_drag_is_not_a_click(win, monkeypatch):
    load(win, "ur5")
    picker = win.link_picker
    monkeypatch.setattr(picker, "link_at", lambda x, y: 2)
    pos = iter([(100, 100), (160, 120)])
    monkeypatch.setattr(picker, "_event_pos", lambda: next(pos))
    got = []
    picker.link_clicked.connect(got.append)
    picker._on_press()
    picker._on_release()
    assert got == []


def test_hover_tints_the_link_and_restores(win):
    load(win, "ur5")
    rv = win.viewport.robot_view
    picker = win.link_picker
    li = next(l for l in rv._visual_links)
    idx = [i for i, l in enumerate(rv._visual_links) if l == li][0]
    before = np.array(rv._visual_actors[idx].prop.color.float_rgb)
    seen = []
    picker.link_hovered.connect(seen.append)
    picker.set_hover(li)
    assert picker.hovered_link == li and seen == [li]
    assert np.allclose(rv._visual_actors[idx].prop.color.float_rgb, HOVER_TINT, atol=0.01)
    picker.set_hover(-1)
    assert np.allclose(rv._visual_actors[idx].prop.color.float_rgb, before, atol=1e-3)
    assert seen == [li, -1]


def test_picking_is_off_while_the_target_pick_mode_is_on(win):
    load(win, "ur5")
    assert not win._target_panel_picking()
    win.target_panel._btn_click.setChecked(True)
    assert win._target_panel_picking()
    win.target_panel._btn_click.setChecked(False)
