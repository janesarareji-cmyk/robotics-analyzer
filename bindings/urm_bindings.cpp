/**
 * bindings/urm_bindings.cpp
 * pybind11 bindings for the URM (Unified Robot Model) C++ types.
 * R2.2: Exposes Joint, Link, RobotModel, DHParams, MDHParams, JointLimits.
 */

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/stl_bind.h>

#include "urm/joint.hpp"
#include "urm/link.hpp"
#include "urm/robot_model.hpp"
#include "urm/robot_model_io.hpp"

PYBIND11_MAKE_OPAQUE(std::vector<urm::Joint>);
PYBIND11_MAKE_OPAQUE(std::vector<urm::Link>);

namespace py = pybind11;

void bind_urm(py::module_& m) {
    py::bind_vector<std::vector<urm::Joint>>(m, "JointVector");
    py::bind_vector<std::vector<urm::Link>>(m, "LinkVector");

    // JointType enum
    py::enum_<urm::JointType>(m, "JointType")
        .value("Revolute",  urm::JointType::Revolute)
        .value("Prismatic", urm::JointType::Prismatic)
        .value("Fixed",     urm::JointType::Fixed)
        .export_values();

    // DHParams
    py::class_<urm::DHParams>(m, "DHParams")
        .def(py::init<>())
        .def_readwrite("a",     &urm::DHParams::a)
        .def_readwrite("alpha", &urm::DHParams::alpha)
        .def_readwrite("d",     &urm::DHParams::d)
        .def_readwrite("theta", &urm::DHParams::theta)
        .def("__repr__", [](const urm::DHParams& p) {
            return "<DHParams a=" + std::to_string(p.a) +
                   " alpha=" + std::to_string(p.alpha) +
                   " d=" + std::to_string(p.d) +
                   " theta=" + std::to_string(p.theta) + ">";
        });

    // MDHParams
    py::class_<urm::MDHParams>(m, "MDHParams")
        .def(py::init<>())
        .def_readwrite("a",     &urm::MDHParams::a)
        .def_readwrite("alpha", &urm::MDHParams::alpha)
        .def_readwrite("d",     &urm::MDHParams::d)
        .def_readwrite("theta", &urm::MDHParams::theta)
        .def("__repr__", [](const urm::MDHParams& p) {
            return "<MDHParams a=" + std::to_string(p.a) +
                   " alpha=" + std::to_string(p.alpha) +
                   " d=" + std::to_string(p.d) +
                   " theta=" + std::to_string(p.theta) + ">";
        });

    // JointLimits
    py::class_<urm::JointLimits>(m, "JointLimits")
        .def(py::init<>())
        .def_readwrite("lower",  &urm::JointLimits::lower)
        .def_readwrite("upper",  &urm::JointLimits::upper)
        .def_readwrite("vel",    &urm::JointLimits::vel)
        .def_readwrite("effort", &urm::JointLimits::effort);

    // Inertia
    py::class_<urm::Inertia>(m, "Inertia")
        .def(py::init<>())
        .def_readwrite("mass",           &urm::Inertia::mass)
        .def_readwrite("com",            &urm::Inertia::com)
        .def_readwrite("inertia_tensor", &urm::Inertia::inertia_tensor);

    // Joint
    py::class_<urm::Joint>(m, "Joint")
        .def(py::init<>())
        .def_readwrite("name",    &urm::Joint::name)
        .def_readwrite("type",    &urm::Joint::type)
        .def_readwrite("dh",      &urm::Joint::dh)
        .def_readwrite("mdh",     &urm::Joint::mdh)
        .def_readwrite("use_mdh", &urm::Joint::use_mdh)
        .def_readwrite("limits",  &urm::Joint::limits)
        .def_readwrite("q",       &urm::Joint::q)
        .def("__repr__", [](const urm::Joint& j) {
            return "<Joint name=" + j.name + " q=" + std::to_string(j.q) + ">";
        });

    // Link
    py::class_<urm::Link>(m, "Link")
        .def(py::init<>())
        .def_readwrite("name",          &urm::Link::name)
        .def_readwrite("geometry_file", &urm::Link::geometry_file)
        .def_readwrite("inertia",       &urm::Link::inertia)
        .def("__repr__", [](const urm::Link& l) {
            return "<Link name=" + l.name + ">";
        });

    // RobotMetadata
    py::class_<urm::RobotMetadata>(m, "RobotMetadata")
        .def(py::init<>())
        .def_readwrite("name",        &urm::RobotMetadata::name)
        .def_readwrite("description", &urm::RobotMetadata::description)
        .def_readwrite("author",      &urm::RobotMetadata::author)
        .def_readwrite("version",     &urm::RobotMetadata::version);

    // RobotModel
    py::class_<urm::RobotModel>(m, "RobotModel")
        .def(py::init<>())
        .def_readwrite("metadata", &urm::RobotModel::metadata)
        .def_readwrite("links",    &urm::RobotModel::links)
        .def_readwrite("joints",   &urm::RobotModel::joints)
        .def("dof",     &urm::RobotModel::dof,
             "Return the number of actuated (non-Fixed) DOF.")
        .def("get_q",   &urm::RobotModel::get_q,
             "Return current joint values as a list of floats.")
        .def("set_q",   &urm::RobotModel::set_q, py::arg("q"),
             "Set joint values from a list of floats.")
        .def("to_json", [](const urm::RobotModel& m, int indent) {
            return urm::to_json(m, indent);
        }, py::arg("indent") = 2,
           "Serialize the robot model to a JSON string.")
        .def_static("from_json", [](const std::string& s) {
            return urm::from_json(s);
        }, py::arg("json_str"),
           "Deserialize a RobotModel from a JSON string.")
        .def("__repr__", [](const urm::RobotModel& m) {
            return "<RobotModel name='" + m.metadata.name +
                   "' dof=" + std::to_string(m.dof()) + ">";
        });

    // Free functions
    m.def("urm_to_json",   &urm::to_json,   py::arg("model"), py::arg("indent") = 2);
    m.def("urm_from_json", &urm::from_json, py::arg("json_str"));
}
