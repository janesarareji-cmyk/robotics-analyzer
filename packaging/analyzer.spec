# -*- mode: python ; coding: utf-8 -*-
# PyInstaller one-folder spec for Robotics Analyzer (R8.5).
# Build: powershell -File packaging/build.ps1   (output: dist/RoboticsAnalyzer/)
# SPECPATH is the packaging/ folder; the repo root is its parent.
import os
from PyInstaller.utils.hooks import collect_submodules, collect_data_files

ROOT = os.path.dirname(SPECPATH)

# Modules that are imported dynamically or only by name (see tests/python/test_packaging.py).
HIDDEN_IMPORTS = [
    "core_bridge",          # C++ extension, .pyd at repo root
    "gpu_bridge",           # optional CUDA extension (CPU fallback when it cannot load)
    "fcl",                  # python-fcl
    "pyvista", "pyvistaqt", "pyqtgraph", "vtk",
    "matplotlib", "matplotlib.backends.backend_agg", "matplotlib.backends.backend_pdf",
    "numpy",
    "xml.etree.ElementTree", "zipfile", "importlib.util",
]
HIDDEN_IMPORTS += collect_submodules("app")
HIDDEN_IMPORTS += collect_submodules("robotics_api")
HIDDEN_IMPORTS += collect_submodules("pyvistaqt")

datas = [
    (os.path.join(ROOT, "examples"), "examples"),
    (os.path.join(ROOT, "plugins_examples"), "plugins_examples"),
    (os.path.join(ROOT, "robots"), "robots"),
]
datas += collect_data_files("pyvista")
datas += collect_data_files("pyqtgraph")
datas += collect_data_files("matplotlib")

binaries = []
for ext in ("core_bridge", "gpu_bridge"):
    for f in os.listdir(ROOT):
        if f.startswith(ext + ".") and f.endswith(".pyd"):
            binaries.append((os.path.join(ROOT, f), "."))

EXCLUDES = [
    "tkinter", "PySide6.QtWebEngineCore", "PySide6.QtWebEngineWidgets", "PySide6.QtWebEngineQuick",
    "PySide6.QtPdf", "PySide6.QtPdfWidgets", "PySide6.Qt3DCore", "PySide6.QtQuick",
    "PySide6.QtQml", "PySide6.QtMultimedia", "pytest", "IPython", "jupyter",
]

a = Analysis(
    [os.path.join(ROOT, "app", "main.py")],
    pathex=[ROOT],
    binaries=binaries,
    datas=datas,
    hiddenimports=HIDDEN_IMPORTS,
    hookspath=[],
    runtime_hooks=[],
    excludes=EXCLUDES,
    noarchive=False,
)
pyz = PYZ(a.pure)
exe = EXE(
    pyz, a.scripts, [],
    exclude_binaries=True,
    name="RoboticsAnalyzer",
    console=True,      # console kept: --smoke prints its result line; GUI start ignores it
    upx=False,
)
coll = COLLECT(exe, a.binaries, a.datas, strip=False, upx=False, name="RoboticsAnalyzer")
