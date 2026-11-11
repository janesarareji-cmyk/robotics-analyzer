# Technology Stack and Compatibility Plan

## Proposed stack

  Area                  Candidate
  --------------------- ---------------------------------
  Desktop UI            Qt 6 / PySide6
  Core engine           C++20
  Python/C++ bridge     pybind11
  Linear algebra        Eigen
  Kinematics/dynamics   Custom C++20/Eigen (Pinocchio deferred — no Windows wheels for Python 3.13)
  GPU compute           CUDA C++ and CuPy
  Optimization          SciPy; PyTorch optional
  CAD kernel            Open CASCADE
  URDF                  Compatible URDF parser
  Motion planning       OMPL
  Collision detection   FCL
  3D visualization      VTK / PyVista
  Graphs                PyQtGraph
  Project data          JSON / SQLite
  Build system          CMake
  Testing               CTest and Python test framework

## GPU target

Primary target: NVIDIA RTX 5060 Laptop GPU, Windows 11.

**Confirmed hardware (OMEN 16):**
- GPU: NVIDIA GeForce RTX 5060 Laptop GPU — **8 GB VRAM** (8151 MiB, confirmed by nvidia-smi)
- GPU Architecture: Blackwell, Compute Capability **12.0**
- Driver: 592.19 (supports CUDA up to 13.1)
- System RAM: 24 GB
- CPU: AMD Ryzen 9 HX series

Design all CUDA batch sizes and memory allocations against the 8 GB VRAM ceiling.
Never hardcode limits; always compute dynamically from available device memory (`cudaMemGetInfo()`).
CUDA Toolkit must be version 12.8 or higher to support SM 12.0 (Blackwell).

## Compatibility gates

Before full implementation, build a small proof-of-compatibility for: 1.
NVIDIA driver and CUDA Toolkit on Windows 11. 2. C++ compiler and CMake
configuration. 3. Qt/PySide6 desktop application. 4. CUDA C++ sample and
CuPy device test. 5. Pinocchio native build and Python binding (deferred — see
docs/hardware-audit.md, R0.8). 6. Open
CASCADE CAD import. 7. OMPL and FCL integration. 8. VTK rendering inside
the desktop UI. 9. pybind11 connection between C++ core and Python UI.

Do not assume all candidate packages have compatible Windows binaries or
compatible versions. Record exact versions, build options and licenses.

## GPU design notes

-   Use CUDA C++ for critical custom kernels.
-   Use CuPy for rapid GPU array development.
-   Use PyTorch only where differentiable computation or optimization is
    needed.
-   Keep validated CPU reference algorithms.
-   Benchmark transfer overhead as well as kernel time.
-   Use batching and chunking for VRAM constraints.
-   Do not promise GPU acceleration for every algorithm.

## Dependency policy

-   Pin tested dependency versions.
-   Maintain a reproducible environment and build instructions.
-   Record third-party licenses and notices.
-   Separate required dependencies from optional integrations.
-   Provide a clean Windows build and packaging test before release.
