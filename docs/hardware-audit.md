# Hardware Audit — R0.1

## GPU
- Model: NVIDIA GeForce RTX 5060 Laptop GPU
- VRAM: 8151 MiB (~8 GB)
- Driver Version: 592.19
- CUDA Version (driver max): 13.1
- Compute Capability: 12.0

## CUDA Toolkit
- Installed: Yes
- Version: 13.5 (V13.5.37)
- Path: C:\Program Files\NVIDIA GPU Computing Toolkit\CUDA\v13.5\bin\nvcc.exe

## Compiler
- MSVC / cl.exe: Found
- Path: C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\VC\Tools\MSVC\14.44.35207\bin\Hostx64\x64\cl.exe
- Visual Studio Version: Visual Studio 2022 Build Tools (MSVC v19.44.35229)

## Build Tools
- CMake: 4.4.4 (Path: C:\Program Files\CMake\bin\cmake.exe)
- Python: 3.13.15
- uv: 0.12.13

## Notes
- NVIDIA driver 592.19 supports CUDA up to 13.1 for driver API.
- CUDA Toolkit 13.5 compiler tools are installed and support Blackwell architecture (Compute Capability 12.0).
- Visual Studio 2022 Build Tools are installed with MSVC toolset 14.44.
- CMake 4.4.4 is installed.
- Machine PATH contains CMake and CUDA paths. A restarted IDE or shell inherits these PATH entries.
- Python 3.13.15 and uv 0.12.13 are installed and functional.

## Pinocchio Decision (R0.8)
- Pinocchio: Not available for Windows Python 3.13 (no win_amd64 wheels on PyPI)
- Source build: Fails (requires Unix `patch` command)
- Decision: R4 implements FK, IK, Jacobians, and dynamics in pure C++20 with Eigen
- Pinocchio status: Deferred — optional validator, revisit in R4-late via Miniconda if needed

## R0 Final Tool Versions
- CMake: 4.4.4
- MSVC: 19.44.35229 (VS 2022 v17.14)
- CUDA Toolkit: 13.5 (SM 12.0 Blackwell confirmed)
- Python: 3.13.15
- uv: 0.12.13
- PySide6: 6.12.0 (Qt 6.12.0)
- pybind11: 2.13.6
- Eigen: 3.4.0
- PyVista: 0.49.2
- pyvistaqt: 0.13.2
- VTK: 9.7.2

