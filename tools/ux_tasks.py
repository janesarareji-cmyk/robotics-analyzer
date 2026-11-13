"""U5 usability measurement: python tools/ux_tasks.py <baseline|after> [screens_dir]

Drives the real MainWindow through 6 tasks with QTest clicks on actual widgets and counts user
actions (click, key, typed field; a combo choice counts 2). Writes docs/ux/<label>.json
(actions and seconds per task) and, when a screens dir is given (real Qt platform), one
screenshot per task.
Importable: ``run_tasks(win, label, shots=None)`` (tests use it offscreen).
"""
from __future__ import annotations

import json
import os
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from PySide6.QtCore import QCoreApplication, QEvent, Qt          # noqa: E402
from PySide6.QtTest import QTest                                   # noqa: E402
from PySide6.QtWidgets import QApplication, QPushButton            # noqa: E402


def pump(sec: float = 0.05) -> None:
    t0 = time.time()
    while True:
        QCoreApplication.processEvents()
        QCoreApplication.sendPostedEvents(None, QEvent.DeferredDelete)
        if time.time() - t0 >= sec:
            return
        time.sleep(0.005)


def pump_until(cond, timeout: float = 60.0) -> bool:
    t0 = time.time()
    while time.time() - t0 < timeout:
        pump(0.02)
        if cond():
            return True
    return False


class Counter:
    """User action counter. Every public method is one kind of user action."""

    def __init__(self, win) -> None:
        self.win = win
        self.n = 0
        self.steps: list[str] = []

    def _count(self, k: int, what: str) -> None:
        self.n += k
        self.steps.append(f"{k}x {what}")

    def click(self, widget, what: str) -> None:
        assert widget is not None and widget.isEnabled(), f"cannot click {what}"
        QTest.mouseClick(widget, Qt.MouseButton.LeftButton)
        self._count(1, "click " + what)
        pump(0.05)

    def click_button(self, parent, text: str) -> None:
        for b in parent.findChildren(QPushButton):
            if b.text() == text and b.isVisible():
                self.click(b, text)
                return
        raise AssertionError(f"button {text!r} not found")

    def menu_pick(self, menu, text: str) -> None:
        for a in menu.actions():
            if a.text().replace("&", "") == text:
                a.trigger()
                self._count(1, "menu " + text)
                pump(0.05)
                return
        raise AssertionError(f"menu item {text!r} not found")

    def typed(self, what: str) -> None:
        self._count(1, "type " + what)

    def choose(self, combo, text: str) -> None:
        combo.setCurrentText(text)
        self._count(2, f"choose {text}")
        pump(0.05)


def _wait_idle(win, timeout: float = 60.0) -> None:
    win.task_runner.wait_all(int(timeout * 1000))
    pump(0.1)


def load_robot(win, c: Counter, stem: str) -> None:
    """Task 1 steps for either UI generation."""
    if hasattr(win, "start_screen") and win.start_screen.isVisible():
        c.click(win.start_screen.card(stem), f"sample card {stem}")
    else:
        from app.sample_robots import list_sample_robots
        c.click(win._sample_button, "toolbar Sample robot")
        title = next(t for t, p in list_sample_robots() if p.parent.name == stem)
        c.menu_pick(win._menu_samples, title)
    _wait_idle(win)
    assert win.project_controller.robot is not None
    pump(0.3)


def open_panel(win, c: Counter, name: str) -> None:
    """Make the dock of view action ``name`` visible with the fewest clicks of this UI."""
    action = getattr(win, "_action_view_" + name)
    dock = next(d for a, d in win._view_action_docks() if a is action)
    if dock.isVisible():
        return
    if hasattr(win, "mode_bar"):
        mode = win.mode_for_dock(name)
        c.click(win.mode_bar.button(mode), f"mode {mode}")
    else:
        c.click(win._side_bar.widgetForAction(action), f"side bar {name}")
    assert dock.isVisible(), f"dock {name} not visible"


def task_load(win, c):
    load_robot(win, c, "ur5")


def task_jog(win, c):
    open_panel(win, c, "joints")
    row = win.joint_panel.rows[1]
    row.spinbox.setValue(row.spinbox.value() + 30.0)
    c.typed("joint 2 +30 deg")
    pump(0.2)


def task_target(win, c):
    import numpy as np
    from app.target import xyz_rpy_from_pose
    open_panel(win, c, "target")
    tp = win.target_panel
    if tp.current_index < 0:
        c.click_button(tp, "New at TCP")
    xyz, rpy = xyz_rpy_from_pose(tp.current_pose())
    for off in ([0.0, 0.0, -50.0], [-50.0, 0.0, 0.0], [0.0, 50.0, 0.0], [50.0, 0.0, 0.0], [0.0, 0.0, 50.0]):
        tp.set_pose(xyz_mm=np.asarray(xyz) * 1000.0 + off, rpy_deg=np.degrees(rpy))
        c.typed(f"target offset {off} mm")
        pump_until(lambda: not tp.is_solving, 30)
        if tp.button_enabled()["play"]:
            break
    c.click_button(tp, "Play to target")
    pump_until(lambda: not tp.is_planning, 30)
    pump(0.3)
    win.sim_panel.stop_trajectory()


def task_plan(win, c):
    win.sim_panel.stop_trajectory()
    c.click(win._objects_button, "toolbar Add object")
    c.menu_pick(win._menu_objects, "Box")
    open_panel(win, c, "planning")
    pp = win.planning_panel
    c.click_button(pp, "Start = current q")
    q = pp.vector("start")
    q[0] += 0.8
    pp.set_vector("goal", q)
    c.typed("goal q1")
    c.click_button(pp, "Plan")
    pump_until(lambda: not pp.is_running, 60)
    pump(0.2)


def task_sim(win, c):
    open_panel(win, c, "simulation")
    sp = win.sim_panel
    c.choose(sp._mode_combo, "dynamic")
    c.click_button(sp, "Start")
    pump(0.5)
    sp.stop()


def task_compare(win, c):
    open_panel(win, c, "compare")
    cp = win.compare_panel
    cp.set_checked(["ur3", "ur5"])
    c._count(2, "tick ur3, ur5")
    c.click_button(cp, "Compare")
    pump_until(lambda: not cp.is_running, 120)
    pump(0.3)


TASKS = [("1_load_robot", task_load), ("2_jog_joint", task_jog), ("3_target_reach", task_target),
         ("4_box_and_plan", task_plan), ("5_dynamic_sim", task_sim), ("6_compare", task_compare)]


def run_tasks(win, label: str, shots: Path | None = None) -> dict:
    result: dict = {"label": label, "tasks": {}}
    for name, fn in TASKS:
        c = Counter(win)
        t0 = time.time()
        fn(win, c)
        dt = time.time() - t0
        result["tasks"][name] = {"actions": c.n, "seconds": round(dt, 2), "steps": c.steps}
        if shots is not None:
            pump(0.6)
            win.viewport.plotter.render()
            pump(0.3)
            win.grab().save(str(shots / f"{name}.png"))
        print(f"{name}: {c.n} actions, {dt:.1f} s", flush=True)
    t = result["tasks"]
    result["task1_plus_3"] = t["1_load_robot"]["actions"] + t["3_target_reach"]["actions"]
    return result


def main() -> int:
    label = sys.argv[1] if len(sys.argv) > 1 else "baseline"
    shots = Path(sys.argv[2]) if len(sys.argv) > 2 else None
    if shots is not None:
        os.environ.setdefault("QT_QPA_PLATFORM", "windows")
        shots.mkdir(parents=True, exist_ok=True)
    else:
        os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
    from app.settings import AppSettings
    AppSettings._ORGANIZATION = "RoboticsAnalyzer_ux_tasks"
    AppSettings._APPLICATION = "RoboticsAnalyzer_ux_tasks"
    from app.main_window import MainWindow
    QApplication.instance() or QApplication([])
    AppSettings()._settings.clear()
    win = MainWindow()
    win.settings._settings.clear()
    win.resize(1600, 950)
    win.show()
    pump(1.0)
    res = run_tasks(win, label, shots)
    if shots is not None:
        win.resize(1400, 900)
        pump(0.8)
        win.grab().save(str(shots / "final_1400.png"))
    out = ROOT / "docs" / "ux" / f"{label}.json"
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(res, indent=2), encoding="utf-8")
    print("task 1+3 =", res["task1_plus_3"], "->", out)
    win.project_controller._is_modified = False
    win.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
