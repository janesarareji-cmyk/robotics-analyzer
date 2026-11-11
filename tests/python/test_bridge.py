"""
tests/python/test_bridge.py
R0.4: Verifies pybind11 bridge builds and links correctly.
"""

import pytest


def test_bridge_import():
    """core_bridge module imports without error."""
    import core_bridge
    assert core_bridge is not None


def test_bridge_add():
    """core_bridge.add returns correct integer sum."""
    import core_bridge
    assert core_bridge.add(2, 3) == 5
    assert core_bridge.add(0, 0) == 0
    assert core_bridge.add(-1, 1) == 0
    assert core_bridge.add(100, 200) == 300


def test_bridge_get_version():
    """core_bridge.get_version returns a non-empty string."""
    import core_bridge
    version = core_bridge.get_version()
    assert isinstance(version, str)
    assert len(version) > 0
    assert "RoboticsAnalyzer" in version


def test_bridge_add_type():
    """core_bridge.add returns an int, not a float."""
    import core_bridge
    result = core_bridge.add(1, 2)
    assert isinstance(result, int)
