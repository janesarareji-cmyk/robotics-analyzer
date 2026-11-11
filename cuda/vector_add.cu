/**
 * vector_add.cu
 * R0.6: GPU vector-add kernel implementation.
 * Verifies CUDA 13.5 + SM 12.0 (Blackwell) compilation and execution.
 */

#include "vector_add.cuh"
#include <cuda_runtime.h>
#include <cstdio>

// GPU kernel: each thread adds one element
__global__ void vector_add_kernel(const float* a, const float* b, float* c, int n) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx < n) {
        c[idx] = a[idx] + b[idx];
    }
}

extern "C" int gpu_vector_add(const float* a, const float* b, float* result, int n) {
    float *d_a = nullptr, *d_b = nullptr, *d_c = nullptr;
    size_t bytes = n * sizeof(float);

    // Allocate device memory
    if (cudaMalloc(&d_a, bytes) != cudaSuccess) return -1;
    if (cudaMalloc(&d_b, bytes) != cudaSuccess) { cudaFree(d_a); return -2; }
    if (cudaMalloc(&d_c, bytes) != cudaSuccess) { cudaFree(d_a); cudaFree(d_b); return -3; }

    // Copy input data to device
    cudaMemcpy(d_a, a, bytes, cudaMemcpyHostToDevice);
    cudaMemcpy(d_b, b, bytes, cudaMemcpyHostToDevice);

    // Launch kernel: 256 threads per block
    int threads = 256;
    int blocks = (n + threads - 1) / threads;
    vector_add_kernel<<<blocks, threads>>>(d_a, d_b, d_c, n);

    // Check for kernel errors
    cudaError_t err = cudaGetLastError();
    if (err != cudaSuccess) {
        fprintf(stderr, "CUDA kernel error: %s\n", cudaGetErrorString(err));
        cudaFree(d_a); cudaFree(d_b); cudaFree(d_c);
        return -4;
    }

    // Synchronize and copy result back
    cudaDeviceSynchronize();
    cudaMemcpy(result, d_c, bytes, cudaMemcpyDeviceToHost);

    // Free device memory
    cudaFree(d_a);
    cudaFree(d_b);
    cudaFree(d_c);

    return 0;
}

extern "C" int cuda_device_info() {
    int device_count = 0;
    cudaGetDeviceCount(&device_count);
    if (device_count == 0) {
        fprintf(stderr, "No CUDA devices found.\n");
        return -1;
    }

    cudaDeviceProp prop;
    cudaGetDeviceProperties(&prop, 0);

    size_t free_mem = 0, total_mem = 0;
    cudaMemGetInfo(&free_mem, &total_mem);

    printf("GPU: %s\n", prop.name);
    printf("Compute Capability: %d.%d\n", prop.major, prop.minor);
    printf("VRAM Total: %.1f MB\n", total_mem / (1024.0 * 1024.0));
    printf("VRAM Free:  %.1f MB\n", free_mem / (1024.0 * 1024.0));

    return 0;
}
