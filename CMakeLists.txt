cmake_minimum_required(VERSION 3.25)
project(RoboticsAnalyzer VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# --- Optional CUDA (gpu_bridge) ---
# RA_ENABLE_CUDA defaults to ON when a CUDA toolkit is found, else OFF.
# With CUDA off, gpu_bridge and the CUDA tests are skipped; core_bridge and the
# C++ tests still build and the app runs on the CPU.
include(CheckLanguage)
check_language(CUDA)
if(CMAKE_CUDA_COMPILER)
    set(_ra_cuda_default ON)
else()
    set(_ra_cuda_default OFF)
endif()
option(RA_ENABLE_CUDA "Build the CUDA module gpu_bridge" ${_ra_cuda_default})

# GPU architecture. setup.ps1 passes the detected one, e.g. -DRA_CUDA_ARCH=120-real.
# SASS only ("-real"): PTX from a newer toolkit may not JIT on an older driver.
# Default keeps the RTX 5060 Laptop (Blackwell SM 12.0) setup.
set(RA_CUDA_ARCH "120-real" CACHE STRING "CUDA architectures for gpu_bridge (e.g. 89-real, native)")

if(RA_ENABLE_CUDA)
    if(NOT CMAKE_CUDA_COMPILER)
        message(FATAL_ERROR "RA_ENABLE_CUDA is ON but no CUDA compiler was found. "
                            "Install the CUDA Toolkit or configure with -DRA_ENABLE_CUDA=OFF.")
    endif()
    set(CMAKE_CUDA_ARCHITECTURES ${RA_CUDA_ARCH})
    enable_language(CUDA)
    set(CMAKE_CUDA_STANDARD 17)
    set(CMAKE_CUDA_STANDARD_REQUIRED ON)
endif()

# Output directories
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

enable_testing()

# --- External dependencies ---
include(FetchContent)

# Disable Eigen internal test suite
set(BUILD_TESTING OFF CACHE BOOL "" FORCE)
set(EIGEN_BUILD_DOC OFF CACHE BOOL "" FORCE)
set(EIGEN_BUILD_PKGCONFIG OFF CACHE BOOL "" FORCE)

# Eigen — header-only linear algebra library
FetchContent_Declare(
    Eigen3
    GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
    GIT_TAG        3.4.0
    GIT_SHALLOW    TRUE
)
FetchContent_MakeAvailable(Eigen3)

# nlohmann_json — JSON for Modern C++
FetchContent_Declare(
    nlohmann_json
    GIT_REPOSITORY https://github.com/nlohmann/json.git
    GIT_TAG        v3.11.3
    GIT_SHALLOW    TRUE
)
set(JSON_BuildTests OFF CACHE BOOL "" FORCE)
set(JSON_Install OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(nlohmann_json)

add_subdirectory(core)
add_subdirectory(bindings)  # before cuda: declares pybind11 for gpu_bridge
if(RA_ENABLE_CUDA)
    add_subdirectory(cuda)
endif()
add_subdirectory(tests/cpp)

message(STATUS "RoboticsAnalyzer v${PROJECT_VERSION} configured")
message(STATUS "C++ Standard: ${CMAKE_CXX_STANDARD}")
message(STATUS "CUDA enabled: ${RA_ENABLE_CUDA}")
if(RA_ENABLE_CUDA)
    message(STATUS "CUDA Architectures: ${CMAKE_CUDA_ARCHITECTURES}")
endif()
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")
