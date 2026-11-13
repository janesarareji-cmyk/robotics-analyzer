/**
 * gpu_manager.cu
 * R5.1: device query, VRAM info, CUDA error policy.
 */
#include "gpu_manager.cuh"

#include <atomic>
#include <mutex>
#include <sstream>

namespace gpu {

static std::atomic<int> g_fake_oom{0};
static std::mutex g_state_mutex;

static std::string err_text(cudaError_t e) {
    return std::string(cudaGetErrorName(e)) + ": " + cudaGetErrorString(e);
}

DeviceInfo query_device() {
    DeviceInfo info;
    try {
        int count = 0;
        cudaError_t e = cudaGetDeviceCount(&count);
        if (e != cudaSuccess) {
            cudaGetLastError();
            info.error = err_text(e);
            return info;
        }
        if (count <= 0) {
            info.error = "no CUDA device";
            return info;
        }
        cudaDeviceProp prop{};
        e = cudaGetDeviceProperties(&prop, 0);
        if (e != cudaSuccess) {
            cudaGetLastError();
            info.error = err_text(e);
            return info;
        }
        info.name = prop.name;
        info.sm_major = prop.major;
        info.sm_minor = prop.minor;
        cudaDriverGetVersion(&info.driver_version);
        cudaRuntimeGetVersion(&info.runtime_version);
        size_t f = 0, t = 0;
        e = cudaMemGetInfo(&f, &t);
        if (e != cudaSuccess) {
            cudaGetLastError();
            info.error = err_text(e);
            return info;
        }
        info.free_bytes = f;
        info.total_bytes = t;
        info.available = true;
    } catch (const std::exception& ex) {
        info.available = false;
        info.error = ex.what();
    } catch (...) {
        info.available = false;
        info.error = "unknown error";
    }
    return info;
}

MemInfo mem_info() {
    MemInfo m;
    size_t f = 0, t = 0;
    cudaError_t e = cudaMemGetInfo(&f, &t);
    if (e != cudaSuccess) {
        cudaGetLastError();
        m.error = err_text(e);
        return m;
    }
    m.ok = true;
    m.free_bytes = f;
    m.total_bytes = t;
    return m;
}

bool is_sticky(cudaError_t e) {
    switch (e) {
        case cudaErrorIllegalAddress:
        case cudaErrorLaunchTimeout:
        case cudaErrorLaunchFailure:
        case cudaErrorHardwareStackError:
        case cudaErrorIllegalInstruction:
        case cudaErrorMisalignedAddress:
        case cudaErrorInvalidAddressSpace:
        case cudaErrorInvalidPc:
        case cudaErrorAssert:
        case cudaErrorECCUncorrectable:
            return true;
        default:
            return false;
    }
}

bool is_oom(cudaError_t e) { return e == cudaErrorMemoryAllocation; }

GpuState& state() {
    static GpuState s;
    return s;
}

void mark_dead(const std::string& msg) {
    std::lock_guard<std::mutex> lk(g_state_mutex);
    GpuState& s = state();
    s.dead = true;
    s.dead_reason = msg;
}

bool is_dead() {
    std::lock_guard<std::mutex> lk(g_state_mutex);
    return state().dead;
}

void set_fake_oom(int n) { g_fake_oom.store(n < 0 ? 0 : n); }

cudaError_t check_alloc() {
    int cur = g_fake_oom.load();
    while (cur > 0) {
        if (g_fake_oom.compare_exchange_weak(cur, cur - 1))
            return cudaErrorMemoryAllocation;
    }
    return cudaSuccess;
}

cudaError_t checked_malloc(void** ptr, size_t bytes) {
    cudaError_t e = check_alloc();
    if (e != cudaSuccess) return e;
    return cudaMalloc(ptr, bytes);
}

static std::string g_fault;

void set_fault(const std::string& name) {
    std::lock_guard<std::mutex> lk(g_state_mutex);
    g_fault = name;
}

bool fault_active(const char* name) {
    std::lock_guard<std::mutex> lk(g_state_mutex);
    return !g_fault.empty() && g_fault == name;
}

void cuda_check(cudaError_t e, const char* expr, const char* file, int line) {
    if (e == cudaSuccess) return;
    std::ostringstream os;
    os << err_text(e) << " [" << expr << " at " << file << ":" << line << "]";
    if (is_sticky(e)) mark_dead(os.str());
    throw CudaError(e, os.str());
}

}  // namespace gpu
