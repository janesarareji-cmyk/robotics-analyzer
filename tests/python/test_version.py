"""U5.3: one version string, used by title, about, reports, CSV meta, plugins."""
import os

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import pytest

import app
from app.main_window import MainWindow
from app.settings import LAYOUT_VERSION, AppSettings


def test_single_version_source():
    assert app.__version__ == "1.1.0"
    from app.sim import export
    assert export.APP_VERSION == app.__version__
    assert MainWindow.APP_VERSION == app.__version__
    assert LAYOUT_VERSION == 3


def test_title_and_about(qapp, monkeypatch):
    monkeypatch.setattr(AppSettings, "_ORGANIZATION", "RoboticsAnalyzer_test_ver")
    monkeypatch.setattr(AppSettings, "_APPLICATION", "RoboticsAnalyzer_test_ver")
    w = MainWindow()
    w.settings._settings.clear()
    assert app.__version__ in w.windowTitle()
    assert "0.1.0" not in w.windowTitle()
    w.close()


def test_report_uses_version():
    from app.reports.report import build_report_data
    import inspect
    assert "APP_VERSION" in inspect.getsource(build_report_data)
