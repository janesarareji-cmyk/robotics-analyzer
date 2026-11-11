/**
 * bindings/dyn_bindings.cpp
 * pybind11 bindings for dynamics (R4.6): DynModel, RNEA, test hooks.
 */

#include <array>
#include <optional>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>

#include "dyn/dynamics.hpp"
#include "kin/compiled_model.hpp"
#include "urm/robot_model.hpp"

namespace py = pybind11;

static Eigen::Vector3d to_vec3(const std::array<double, 3>& g) {
    return Eigen::Vector3d(g[0], g[1], g[2]);
}

void bind_dyn(py::module_& m) {
    py::class_<dyn::DynModel>(m, "DynModel")
        .def_readonly("n_dof", &dyn::DynModel::n_dof)
        .def_readonly("n_links", &dyn::DynModel::n_links);

    const std::array<double, 3> g0{0.0, 0.0, -9.81};

    m.def("build_dyn_model", [](const urm::RobotModel& model, const kin::CompiledModel& cm) {
        return dyn::build_dyn_model(model, cm);
    }, py::arg("model"), py::arg("cm"),
       "Build the dynamics model (spatial inertias, motion subspaces) from a model and its compiled form.");

    py::class_<dyn::Friction>(m, "Friction")
        .def(py::init<>())
        .def_readwrite("viscous", &dyn::Friction::viscous)
        .def_readwrite("coulomb", &dyn::Friction::coulomb)
        .def_readwrite("eps", &dyn::Friction::eps);

    m.def("inverse_dynamics", [](const dyn::DynModel& dm, const std::vector<double>& q,
                                 const std::vector<double>& qd, const std::vector<double>& qdd,
                                 const std::array<double, 3>& gravity,
                                 const std::optional<dyn::Friction>& friction) {
        return dyn::inverse_dynamics(dm, q, qd, qdd, to_vec3(gravity), friction ? &*friction : nullptr);
    }, py::arg("dm"), py::arg("q"), py::arg("qd"), py::arg("qdd"), py::arg("gravity") = g0,
       py::arg("friction") = py::none(),
       "RNEA: actuator effort per DOF for (q, qd, qdd), plus friction if given. gravity is the world vector.");

    m.def("mass_matrix", [](const dyn::DynModel& dm, const std::vector<double>& q) {
        return dyn::mass_matrix(dm, q);
    }, py::arg("dm"), py::arg("q"), "Joint-space mass matrix (n_dof x n_dof).");

    m.def("friction_torque", [](const dyn::DynModel& dm, const dyn::Friction& f,
                                const std::vector<double>& qd) {
        return dyn::friction_torque(dm, f, qd);
    }, py::arg("dm"), py::arg("friction"), py::arg("qd"), "viscous*qd + coulomb*tanh(qd/eps) per DOF.");

    m.def("forward_dynamics", [](const dyn::DynModel& dm, const std::vector<double>& q,
                                 const std::vector<double>& qd, const std::vector<double>& tau,
                                 const std::array<double, 3>& gravity,
                                 const std::optional<dyn::Friction>& friction,
                                 const std::optional<std::vector<double>>& armature) {
        return dyn::forward_dynamics(dm, q, qd, tau, to_vec3(gravity), friction ? &*friction : nullptr,
                                     armature ? &*armature : nullptr);
    }, py::arg("dm"), py::arg("q"), py::arg("qd"), py::arg("tau"), py::arg("gravity") = g0,
       py::arg("friction") = py::none(), py::arg("armature") = py::none(),
       "qdd = M^-1 (tau - friction - bias). RuntimeError if M is not positive definite.");

    m.def("gravity_torque", [](const dyn::DynModel& dm, const std::vector<double>& q,
                               const std::array<double, 3>& gravity) {
        return dyn::gravity_torque(dm, q, to_vec3(gravity));
    }, py::arg("dm"), py::arg("q"), py::arg("gravity") = g0, "g(q) = RNEA(q, 0, 0).");

    m.def("bias_torque", [](const dyn::DynModel& dm, const std::vector<double>& q,
                            const std::vector<double>& qd, const std::array<double, 3>& gravity) {
        return dyn::bias_torque(dm, q, qd, to_vec3(gravity));
    }, py::arg("dm"), py::arg("q"), py::arg("qd"), py::arg("gravity") = g0,
       "RNEA(q, qd, 0) (includes gravity).");

    m.def("coriolis_torque", [](const dyn::DynModel& dm, const std::vector<double>& q,
                                const std::vector<double>& qd, const std::array<double, 3>& gravity) {
        return dyn::coriolis_torque(dm, q, qd, to_vec3(gravity));
    }, py::arg("dm"), py::arg("q"), py::arg("qd"), py::arg("gravity") = g0,
       "C(q, qd) qd = bias - gravity.");

    m.def("dyn_frames", [](const dyn::DynModel& dm, const std::vector<double>& q) {
        py::list out;
        for (const auto& f : dyn::dyn_frames(dm, q)) out.append(Eigen::Matrix4d(f.matrix()));
        return out;
    }, py::arg("dm"), py::arg("q"),
       "Test hook: world frames from the dynamics transforms (must equal fk).");

    m.def("dyn_kinetic_energy_spatial", [](const dyn::DynModel& dm, const std::vector<double>& q,
                                           const std::vector<double>& qd) {
        return dyn::kinetic_energy_spatial(dm, q, qd);
    }, py::arg("dm"), py::arg("q"), py::arg("qd"),
       "Test hook: 0.5 * sum v_k^T I_k v_k.");
}
