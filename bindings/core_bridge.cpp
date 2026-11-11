/**
 * core_bridge.cpp
 * pybind11 bridge — exposes core C++ functions to Python.
 * R0.4: Minimal bridge to verify pybind11 builds and links correctly.
 * R2.2: Wires URM bindings into core_bridge.
 * R2.5: Wires Kinematics (FK) bindings into core_bridge.
 */

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <stdexcept>
#include <string>

#include "compute/budget.hpp"

namespace py = pybind11;

// Forward declare from core_lib
std::string get_version();

// Forward declare from urm_bindings
void bind_urm(py::module_& m);

// Forward declare from kin_bindings
void bind_kin(py::module_& m);

// Forward declare from dyn_bindings
void bind_dyn(py::module_& m);

// Simple arithmetic function for bridge verification
int add(int a, int b) {
    return a + b;
}

PYBIND11_MODULE(core_bridge, m) {
    m.doc() = "Robotics Analyzer core C++ bridge (pybind11)";

    m.def("add", &add,
          py::arg("a"), py::arg("b"),
          "Add two integers. Returns a + b.");

    m.def("get_version", &get_version,
          "Return the core library version string.");

    // R2.2: URM bindings
    bind_urm(m);

    // R2.5: Kinematics bindings
    bind_kin(m);

    // R4.6: Dynamics bindings
    bind_dyn(m);

    // R5.1: GPU chunk-size rule (pure function, no CUDA)
    m.def("chunk_size",
          [](long long free_bytes, long long user_budget_bytes,
             long long persistent_bytes, long long bytes_per_sample,
             double headroom, long long reserve_bytes, long long min_chunk,
             long long max_chunk) {
              try {
                  return compute::chunk_size(free_bytes, user_budget_bytes,
                                             persistent_bytes, bytes_per_sample,
                                             headroom, reserve_bytes, min_chunk,
                                             max_chunk);
              } catch (const std::invalid_argument& e) {
                  throw py::value_error(e.what());
              }
          },
          py::arg("free_bytes"), py::arg("user_budget_bytes"),
          py::arg("persistent_bytes"), py::arg("bytes_per_sample"),
          py::arg("headroom") = 0.6,
          py::arg("reserve_bytes") = 256LL * 1024 * 1024,
          py::arg("min_chunk") = 65536, py::arg("max_chunk") = 4194304,
          "GPU chunk size in samples (0 = use CPU).");
}
