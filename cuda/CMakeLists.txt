# CUDA compute library for Robotics Analyzer
# R0.6: Vector-add kernel — verifies GPU execution on SM 12.0

add_library(cuda_lib STATIC
    vector_add.cu
)

target_include_directories(cuda_lib PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

set_target_properties(cuda_lib PROPERTIES
    CUDA_SEPARABLE_COMPILATION ON
    CUDA_RESOLVE_DEVICE_SYMBOLS ON
)

# R5.1: GPU manager (device query, error policy)
target_sources(cuda_lib PRIVATE gpu_manager.cu fk_kernel.cu workspace_kernel.cu ik_kernel.cu)
find_package(CUDAToolkit REQUIRED)
target_link_libraries(cuda_lib PUBLIC CUDA::cudart_static)

# R5.1: optional pybind11 module gpu_bridge (pybind11 declared in bindings/)
pybind11_add_module(gpu_bridge gpu_bridge.cpp workspace_gpu.cpp ik_gpu.cpp)
target_link_libraries(gpu_bridge PRIVATE cuda_lib core_lib)
target_compile_features(gpu_bridge PRIVATE cxx_std_20)

add_custom_command(TARGET gpu_bridge POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy
        $<TARGET_FILE:gpu_bridge>
        ${CMAKE_SOURCE_DIR}/$<TARGET_FILE_NAME:gpu_bridge>
    COMMENT "Copying gpu_bridge to project root"
)

# R5.3: CCCL (cub/thrust in CUDA 13.5) needs the standard conforming MSVC preprocessor.
if(MSVC)
    target_compile_options(cuda_lib PRIVATE $<$<COMPILE_LANGUAGE:CUDA>:-Xcompiler=/Zc:preprocessor>)
endif()

# R5.3: shared host/device headers (compute/philox.hpp); cuda_lib does not link core_lib.
target_include_directories(cuda_lib PRIVATE ${CMAKE_SOURCE_DIR}/core)
