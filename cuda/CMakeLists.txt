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
