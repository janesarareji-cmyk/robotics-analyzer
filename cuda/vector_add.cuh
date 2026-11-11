/**
 * vector_add.cuh
 * R0.6: GPU vector-add kernel interface.
 * GPU architecture: Blackwell SM 12.0
 */
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/**
 * gpu_vector_add — adds two float arrays on GPU.
 * @param a       Input array A (host pointer, length n)
 * @param b       Input array B (host pointer, length n)
 * @param result  Output array (host pointer, length n) — allocated by caller
 * @param n       Number of elements
 * @return        0 on success, non-zero on CUDA error
 */
int gpu_vector_add(const float* a, const float* b, float* result, int n);

/**
 * cuda_device_info — prints GPU name and available memory to stdout.
 * @return  0 on success, non-zero on CUDA error
 */
int cuda_device_info();

#ifdef __cplusplus
}
#endif
