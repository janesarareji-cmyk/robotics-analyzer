"""Walkthrough screenshots: python tools/walkthrough_shots.py [outdir]

Real platform window, isolated settings. One PNG per start screen, mode, dock, menu, overlay, demo step.
Used by docs/walkthrough.md.
"""
from __future__ import annotations

import os
import sys
import time
import traceback
from pathlib import Path

os.environ["QT_QPA_PLATFORM"] = "windows"
ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

import numpy as np                                            # noqa: E402
from PySide6.QtCore import QCoreApplication, QEvent, QPoint   # noqa: E402
from PySide6.QtWidgets import QAbstractButton, QApplication   # noqa: E402

from app.settings import AppSettings                          # noqa: E402

AppSettings._ORGANIZATION = "RoboticsAnalyzer_shots_wt"
AppSettings._APPLICATION = "RoboticsAnalyzer_shots_wt"

from app.kin_utils import current_q                           # noqa: E402
from app.main_window import MainWindow                        # noqa: E402
from app.sim.trajectory import Trajectory, Waypoint           # noqa: E402
from tests.python.test_live_blocking import SAMPLES, load, put_q   # noqa: E402

COUNT = 0


def pump(sec: float = 0.6) -> None:
    t0 = time.time()
    while time.time() - t0 < sec:
        QCoreApplication.processEvents()
        QCoreApplication.sendPostedEvents(None, QEvent.DeferredDelete)
        time.sleep(0.01)


def pump_until(cond, timeout=120.0) -> bool:
    t0 = time.time()
    while time.time() - t0 < timeout:
        pump(0.05)
        if cond():
            return True
    return cond()


def shot(win, out: Path, name: str) -> None:
    global COUNT
    pump(0.8)
    win.viewport.plotter.render()
    pump(0.4)
    win.grab().save(str(out / f"{name}.png"))
    COUNT += 1
    print("saved", name, flush=True)


def screen_shot(win, out: Path, name: str) -> None:
    """Grab from the screen so open popup menus are visible."""
    global COUNT
    pump(0.6)
    g = win.frameGeometry()
    win.screen().grabWindow(0, g.x(), g.y(), g.width(), g.height()).save(str(out / f"{name}.png"))
    COUNT += 1
    print("saved", name, flush=True)


def _menu_action(win, title):
    return next((a for a in win.menuBar().actions() if a.text().replace("&", "") == title), None)


def menu_shot(win, out: Path, name: str, title: str) -> None:
    bar = win.menuBar()
    act = _menu_action(win, title)
    menu = act.menu()
    r = bar.actionGeometry(act)
    menu.popup(bar.mapToGlobal(QPoint(r.left(), r.bottom())))
    pump(0.5)
    screen_shot(win, out, name)
    menu.close()
    pump(0.3)


def submenu_shot(win, out, name, path):
    bar = win.menuBar()
    act = _menu_action(win, path[0])
    menu = act.menu()
    r = bar.actionGeometry(act)
    menu.popup(bar.mapToGlobal(QPoint(r.left(), r.bottom())))
    pump(0.3)
    sub = next(a for a in menu.actions() if a.menu() and a.text().replace("&", "") == path[1])
    ar = menu.actionGeometry(sub)
    sub.menu().popup(menu.mapToGlobal(QPoint(ar.right(), ar.top())))
    pump(0.4)
    screen_shot(win, out, name)
    sub.menu().close()
    menu.close()
    pump(0.3)


def dock_of(win, view):
    act = getattr(win, "_action_view_" + view)
    return next(d for a, d in win._view_action_docks() if a is act)


def dock_shot(win, out, name, view):
    dock = dock_of(win, view)
    for _a, d in win._view_action_docks():
        if d is not dock and d is not dock_of(win, "log"):
            d.hide()
    pump(0.3)
    dock.show()
    dock.raise_()
    pump(0.5)
    shot(win, out, name)


def step(label, fn):
    try:
        fn()
    except Exception:
        print("FAILED", label, flush=True)
        traceback.print_exc()


def main() -> int:
    out = Path(sys.argv[1]) if len(sys.argv) > 1 else ROOT / "docs" / "screens" / "walkthrough"
    out.mkdir(parents=True, exist_ok=True)
    QApplication.instance() or QApplication([])
    AppSettings()._settings.clear()
    win = MainWindow()
    win.settings._settings.clear()
    win.resize(1600, 950)
    win.move(20, 20)
    win.show()
    pump(1.2)
    step("tour", lambda: shot(win, out, "00_first_run_tour"))
    win._tour._end(True)
    pump(0.5)
    shot(win, out, "01_start_screen")

    model = load(win, "ur5")
    win.viewport.reset_camera()
    pump(1.0)
    shot(win, out, "02_ur5_loaded")

    for i, m in enumerate(["Model", "Move", "Target", "Plan", "Simulate", "Analyze"], start=1):
        step(m, lambda m=m, i=i: (win.set_mode(m), pump(0.8), shot(win, out, f"03_mode_{i}_{m.lower()}")))

    win.set_mode("Move")
    docks = [
        ("model", "10_dock_model_tree"), ("props", "11_dock_properties"), ("dh", "12_dock_dh_editor"),
        ("physical", "13_dock_physical"), ("joints", "14_dock_joints_jog"), ("matrices", "15_dock_matrices"),
        ("collision", "16_dock_collision"), ("target", "17_dock_target"), ("curve", "18_dock_curve"),
        ("planning", "19_dock_planning"), ("ghosts", "20_dock_ghosts"), ("simulation", "21_dock_simulation"),
        ("graphs", "22_dock_graphs"), ("workspace", "23_dock_workspace"), ("ik", "24_dock_ik"),
        ("analysis", "25_dock_analysis"), ("compare", "26_dock_compare"), ("overlays", "27_dock_overlays"),
        ("performance", "28_dock_performance"), ("script", "29_dock_script"), ("plugins", "30_dock_plugins"),
        ("log", "31_dock_log"), ("import_report", "32_dock_import_report"),
    ]
    for view, name in docks:
        step(name, lambda v=view, n=name: dock_shot(win, out, n, v))

    win.set_mode("Move")
    for v in ("script", "plugins", "performance", "import_report"):
        dock_of(win, v).hide()
    pump(0.5)
    put_q(win, [0.3, -1.2, 1.4, -0.5, 0.8, 0.2])
    win.viewport.reset_camera()
    shot(win, out, "40_move_mode_clean")

    vb = win._view_buttons
    for nm in ("Front", "Top", "Side", "Iso"):
        def f(nm=nm):
            btn = next(b for b in vb.findChildren(QAbstractButton) if b.text().replace("&", "") == nm)
            btn.click()
            shot(win, out, f"41_view_{nm.lower()}")
        step(nm, f)
    step("fit", lambda: [b.click() for b in vb.findChildren(QAbstractButton) if b.text() == "Fit"])

    def playback():
        q0 = np.array(current_q(model))
        q1 = q0 + np.array([0.7, -0.5, 0.5, 0.4, -0.3, 0.2])
        traj = Trajectory([Waypoint(0.0, q0.tolist()), Waypoint(1.2, q1.tolist())], interpolation="cubic")
        win.sim_panel.load_trajectory(traj)
        win.sim_panel.set_mode("kinematic")
        win.sim_panel.seek_trajectory(0.5)
        pump(0.6)
        shot(win, out, "42_playback_bar")
    step("playback", playback)

    for i, t in enumerate(["File", "Edit", "View", "Robot", "Help"], start=1):
        step(t, lambda t=t, i=i: menu_shot(win, out, f"50_menu_{i}_{t.lower()}", t))
    step("samples", lambda: submenu_shot(win, out, "55_menu_file_samples", ["File", "Open Sample Robot"]))
    step("import", lambda: submenu_shot(win, out, "56_menu_file_import", ["File", "Import"]))
    step("export", lambda: submenu_shot(win, out, "57_menu_file_export", ["File", "Export"]))
    step("viewrobot", lambda: submenu_shot(win, out, "58_menu_view_robot", ["View", "Robot"]))
    step("overlaymenu", lambda: submenu_shot(win, out, "59_menu_view_overlays", ["View", "Analysis overlays"]))

    def helpdlg():
        win.show_help()
        pump(0.8)
        win._help_dialog.grab().save(str(out / "60_help_dialog.png"))
        win._help_dialog.close()
    step("help", helpdlg)
    step("tour2", lambda: (win.start_tour(), pump(0.8), shot(win, out, "61_tour_card"), win._tour._end(True)))
    step("focus", lambda: (win.set_focus_mode(True), pump(0.8), shot(win, out, "62_focus_3d"),
                           win.set_focus_mode(False)))

    m = win.overlays
    win.set_mode("Analyze")
    put_q(win, [0.3, -1.2, 1.4, -0.5, 0.8, 0.2])
    win.viewport.reset_camera()

    def reach():
        m.set_enabled("reach", True)
        assert pump_until(lambda: m.data("reach") is not None)
        shot(win, out, "70_overlay_reach")
        m.set_enabled("reach", False)
    step("reach", reach)

    def sing():
        m.set_enabled("singularity", True)
        assert pump_until(lambda: m.data("singularity") is not None)
        shot(win, out, "71_overlay_singularity")
        m.set_enabled("singularity", False)
    step("sing", sing)

    def load_ov():
        win.set_mode("Move")
        m.set_enabled("load", True)
        put_q(win, [0.3, -0.3, 0.4, -0.5, 0.8, 0.2])
        pump(0.3)
        put_q(win, [0.3, -0.3, 0.4, -0.5, 0.8, 0.2])
        shot(win, out, "72_overlay_load")
        m.set_enabled("load", False)
    step("load", load_ov)

    # second robot through the start-screen path: Franka Panda with sample targets and the demo tour
    def panda():
        win.project_controller._is_modified = False
        win.project_controller.close_project()
        pump(1.0)
        win.import_sample_file(str(SAMPLES["panda"][1]))
        assert win.task_runner.wait_all(30000)
        pump(1.5)
        win.set_mode("Target")
        win.viewport.reset_camera()
        shot(win, out, "80_panda_targets_hint")
    step("panda", panda)

    def demo():
        win.start_demo()
        for i in range(5):
            pump(3.0)
            shot(win, out, f"81_demo_step_{i}")
        win._target_panel.stop_demo() if hasattr(win._target_panel, "stop_demo") else None
    step("demo", demo)

    print("COUNT", COUNT, flush=True)
    win.project_controller._is_modified = False
    win.sim_dock.hide()
    win.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
