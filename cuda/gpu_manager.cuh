/**
 * gpu_manager.cuh
 * R5.1: device query, VRAM info, CUDA error policy.
 */
#pragma once

#include <cuda_runtime.h>
#include <cstddef>
#include <stdexcept>
#include <string>

namespace gpu {

struct DeviceInfo {
    bool available = false;
    std::string name;
    int sm_major = 0;
    int sm_minor = 0;
    size_t total_bytes = 0;
    size_t free_bytes = 0;
    int driver_version = 0;
    int runtime_version = 0;
    std::string error;
};

struct MemInfo {
    bool ok = false;
    size_t free_bytes = 0;
    size_t total_bytes = 0;
    std::string error;
};

/// Never throws. No device or driver error -> available = false, error set.
DeviceInfo query_device();
/// Never throws. On failure ok = false.
MemInfo mem_info();

class CudaError : public std::runtime_error {
public:
    CudaError(cudaError_t code, const std::string& msg)
        : std::runtime_error(msg), code_(code) {}
    cudaError_t code() const { return code_; }
private:
    cudaError_t code_;
};

/// Sticky errors corrupt the context: GPU unusable for the process.
bool is_sticky(cudaError_t e);
/// Out-of-memory (not sticky): free, halve the chunk, retry.
bool is_oom(cudaError_t e);

/// Process-wide GPU state.
struct GpuState {
    bool dead = false;
    std::string dead_reason;
};
GpuState& state();
void mark_dead(const std::string& msg);
bool is_dead();

/// Test hook: the next n allocations (check_alloc / checked_malloc) report OOM.
void set_fake_oom(int n);
/// Consumes one fake OOM if armed; returns cudaErrorMemoryAllocation, else cudaSuccess.
cudaError_t check_alloc();
/// cudaMalloc wrapper honoring the fake OOM hook (returns the error code, no throw).
cudaError_t checked_malloc(void** ptr, size_t bytes);

/// Throws CudaError with name and string; marks the GPU dead for sticky errors.
void cuda_check(cudaError_t e, const char* expr, const char* file, int line);

}  // namespace gpu

#define CUDA_CHECK(expr) ::gpu::cuda_check((expr), #expr, __FILE__, __LINE__)
