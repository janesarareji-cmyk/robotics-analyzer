# Clean machine test (R8.5)

## Done (dev machine, scrubbed environment)

Built `dist/RoboticsAnalyzer` (498 MiB, zip 170 MiB, 117 s, PyInstaller 6.22.3). Ran
`RoboticsAnalyzer.exe --smoke` via `packaging/scrub_smoke.py`: environment holds only SystemRoot,
TEMP/TMP, QT_QPA_PLATFORM, and `PATH=%SystemRoot%\System32;%SystemRoot%` (no Python, no repo),
cwd = dist folder.

| Case | Result |
|------|--------|
| default | exit 0, `fk_tcp=[2.0, 0.0, 0.0]`, `backend=cuda` (driver present) |
| `ANALYZER_FORCE_CPU=1` | exit 0, `backend=cpu reason=forced by env ANALYZER_FORCE_CPU` |
| `gpu_bridge.pyd` removed from dist | exit 0, `backend=cpu reason=gpu_bridge not importable` |

Offscreen only. The visible window and 3D viewport (OpenGL) were NOT checked in the frozen build.
The test ran on a machine that still has Python and the repo installed (not a new user, not a new
machine). This is NOT a clean-machine pass.

## Left for the user

1. Copy `dist/RoboticsAnalyzer-win64.zip` to a Windows Sandbox or a fresh Windows user account (or
   a second PC without Python/VS/CUDA).
2. Unzip, run `install.ps1` (or run the exe in place). Start `RoboticsAnalyzer.exe`: main window,
   viewport, import `planar_2r.urdf`, move a joint.
3. `RoboticsAnalyzer.exe --smoke` in a terminal: expect `SMOKE OK`.
4. With NVIDIA driver: Workspace panel shows GPU. Without: CPU fallback, no crash.
5. If the exe fails to start: install the VC++ redistributable (x64) and retry; report the error.
6. Optional: install Inno Setup 6 and build `packaging/installer.iss` (to be written).
