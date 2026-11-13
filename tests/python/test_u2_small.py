"""U2.1 small fixes: import report dock, start pose, frames default, project name."""

import os

import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtCore import QCoreApplication

from app.importers.urdf import import_urdf
from app.kin_utils import current_q, dof_limits
from app.main_window import MainWindow
from app.sample_robots import import_display_name, list_sample_robots
from app.settings import AppSettings

SAMPLES = {p.parent.name: (t, p) for t, p in list_sample_robots()}


@pytest.fixture
def win(qapp, monkeypatch):
    monkeypatch.setattr(AppSettings, "_ORGANIZATION", "RoboticsAnalyzer_test_u2")
    monkeypatch.setattr(AppSettings, "_APPLICATION", "RoboticsAnalyzer_test_u2")
    w = MainWindow()
    w.settings._settings.clear()
    yield w
    w.project_controller._is_modified = False
    w.close()


def _load(win, path):
    res = import_urdf(str(path))
    win._import_path = str(path)
    win._on_urdf_import_result(res)
    QCoreApplication.processEvents()
    return res


def test_panda_start_pose_in_range():
    res = import_urdf(str(SAMPLES["panda"][1]))
    q = current_q(res.model)
    lo, hi = dof_limits(res.model)
    for i, (a, b, v) in enumerate(zip(lo, hi, q)):
        assert a <= v <= b
        if a <= 0.0 <= b:
            assert v == 0.0
        else:
            assert v == pytest.approx(0.5 * (a + b))
    # panda joint4 range is [-3.07, -0.07]: 0 is outside -> middle, not the limit
    assert q[3] == pytest.approx(-1.57, abs=1e-2)


def test_dock_hidden_for_warnings_shown_for_errors(win, tmp_path):
    win.show()
    win.import_report_dock.hide()
    # a sample with warnings only
    res = _load(win, SAMPLES["xarm5"][1])
    n_warn = len(res.report.by_severity("warning"))
    if n_warn:
        assert not win.import_report_dock.isVisible()
        assert f"Imported with {n_warn} warnings" in win._status_label.text()
    # broken urdf -> error -> dock visible
    bad = tmp_path / "bad.urdf"
    bad.write_text("<robot name='x'><joint name='j' type='revolute'/></robot>")
    res = import_urdf(str(bad))
    assert res.report.has_errors()
    win._import_path = str(bad)
    win._on_urdf_import_result(res)
    assert win.import_report_dock.isVisible()


def test_frames_default_off_with_visuals(win):
    _load(win, SAMPLES["ur5"][1])
    assert win.project_controller.current_project.visuals
    assert not win._action_show_frames.isChecked()
    assert win.viewport.robot_view._show_frames is False
    win._action_show_frames.setChecked(True)      # user toggle still works
    assert win.viewport.robot_view._show_frames is True


def test_frames_on_without_visuals(win, tmp_path):
    urdf = tmp_path / "r.urdf"
    urdf.write_text(
        "<robot name='r'><link name='a'/><link name='b'/>"
        "<joint name='j' type='revolute'><parent link='a'/><child link='b'/>"
        "<axis xyz='0 0 1'/><limit lower='-1' upper='1' effort='1' velocity='1'/>"
        "<origin xyz='0 0 1'/></joint></robot>"
    )
    win._action_show_frames.setChecked(False)
    _load(win, urdf)
    assert not win.project_controller.current_project.visuals
    assert win._action_show_frames.isChecked()


def test_names(win, tmp_path):
    t, p = SAMPLES["xarm5"]
    assert import_display_name(str(p), "UF_ROBOT") == t
    assert import_display_name(str(tmp_path / "my_arm.urdf"), "UF_ROBOT") == "my_arm"
    assert import_display_name(str(tmp_path / "my_arm.urdf"), "") == "my_arm"
    assert import_display_name(str(tmp_path / "my_arm.urdf"), "robot") == "my_arm"
    assert import_display_name(str(tmp_path / "my_arm.urdf"), "Fancy") == "Fancy"
    _load(win, p)
    assert win.project_controller.current_project.name == t
