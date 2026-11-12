# Packaging (R8.5)

Tool: PyInstaller 6.22.3 (one-folder), Python 3.13.15, PySide6 6.12.0, VTK 9.7.2, PyVista 0.49.2,
pyqtgraph 0.14.0, python-fcl 0.7.0.11, numpy 2.5.4. Nuitka not tried (PyInstaller worked).

## Build

    powershell -ExecutionPolicy Bypass -File packaging/build.ps1

Output (untracked, `dist/` is in `.gitignore`):
`dist/RoboticsAnalyzer/` (exe + `_internal/`) and `dist/RoboticsAnalyzer-win64.zip`.
Spec: `packaging/analyzer.spec` (hidden imports: app/robotics_api submodules, core_bridge,
gpu_bridge, fcl, pyvista, pyvistaqt, pyqtgraph, vtk, matplotlib Agg/PDF). Excluded: tkinter,
Qt WebEngine/Pdf/Qt3D/Quick/QML/Multimedia, pytest.

Measured on the dev machine (Windows 11, Ryzen 9 8940HX, RTX 5060): build 117 s, folder 498 MiB,
zip 170 MiB.

## CUDA runtime

`gpu_bridge` links `CUDA::cudart_static` (cuda/CMakeLists.txt): no CUDA runtime DLL is bundled and
none is needed. Only the NVIDIA driver (user's, `nvcuda.dll`) is required for the GPU path. Without
it, or without `gpu_bridge`, the app falls back to the CPU (reason `gpu_bridge not importable` /
`no device`). `ANALYZER_FORCE_CPU=1` forces the CPU.

## Smoke start

`RoboticsAnalyzer.exe --smoke` (add `QT_QPA_PLATFORM=offscreen` for headless; set by default):
creates MainWindow, imports the bundled `examples/planar_2r.urdf`, runs one FK, prints one line
(`SMOKE OK python=.. qt=.. vtk=.. fk_tcp=[..] backend=.. reason=..`), exit 0 or 1.
`packaging/scrub_smoke.py` runs it with a scrubbed environment.

## Installer

- Zip + `install.ps1` (per-user copy to `%LOCALAPPDATA%\RoboticsAnalyzer`, Start menu shortcut, no
  admin, no PATH change). Delivered.
- Inno Setup: `ISCC.exe` not found on this machine. Not installed (system software). Manual step
  for the user; `packaging/installer.iss` is not written yet.

## Known limits

- Console window stays (needed for `--smoke` output).
- The script runner uses the frozen stdlib only; user scripts importing modules that are not bundled
  fail with ImportError.
- Plugins are loaded from files at run time; their third-party imports must already be bundled.
- Not code-signed: Windows SmartScreen will warn.
