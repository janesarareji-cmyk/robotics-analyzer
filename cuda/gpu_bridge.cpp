/**
 * gpu_bridge.cpp
 * R5.1: optional pybind11 module for GPU compute. core_bridge never links CUDA.
 * Imports fine without a GPU (device_info reports available = False).
 */
#include <pybind11/pybind11.h>

#include "gpu_manager.cuh"

namespace py = pybind11;

PYBIND11_MODULE(gpu_bridge, m) {
    m.doc() = "Robotics Analyzer GPU bridge (CUDA, optional)";

    m.def("device_info", [] {
        gpu::DeviceInfo d = gpu::query_device();
        py::dict r;
        r["available"] = d.available;
        r["name"] = d.name;
        r["sm_major"] = d.sm_major;
        r["sm_minor"] = d.sm_minor;
        r["total_bytes"] = static_cast<unsigned long long>(d.total_bytes);
        r["free_bytes"] = static_cast<unsigned long long>(d.free_bytes);
        r["driver_version"] = d.driver_version;
        r["runtime_version"] = d.runtime_version;
        r["error"] = d.error;
        return r;
    }, "Return a dict describing CUDA device 0. Never raises.");

    m.def("mem_info", [] {
        gpu::MemInfo mi = gpu::mem_info();
        py::dict r;
        r["ok"] = mi.ok;
        r["free_bytes"] = static_cast<unsigned long long>(mi.free_bytes);
        r["total_bytes"] = static_cast<unsigned long long>(mi.total_bytes);
        r["error"] = mi.error;
        return r;
    }, "Return free/total VRAM in bytes. Never raises.");

    m.def("is_dead", [] { return gpu::is_dead(); },
          "True after a sticky CUDA error marked the GPU dead.");

    m.def("set_fake_oom", [](int n) { gpu::set_fake_oom(n); }, py::arg("n"),
          "Test hook: the next n GPU allocations report out-of-memory.");
}
