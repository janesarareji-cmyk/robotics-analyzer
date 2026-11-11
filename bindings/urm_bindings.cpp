/**
 * bindings/urm_bindings.cpp
 * pybind11 bindings for the URM (Unified Robot Model) C++ types.
 * R3.1: Exposes JointKinematics, Mimic, tree links/joints, tcp_link, q_index_map.
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

    // JointKinematics enum
    py::enum_<urm::JointKinematics>(m, "JointKinematics")
        .value("DH",         urm::JointKinematics::DH)
        .value("OriginAxis", urm::JointKinematics::OriginAxis)
        .export_values();

    // DHConvention enum
    py::enum_<urm::DHConvention>(m, "DHConvention")
        .value("Standard", urm::DHConvention::Standard)
        .value("Modified", urm::DHConvention::Modified)
        .export_values();

    // Pose
    py::class_<urm::Pose>(m, "Pose")
        .def(py::init<>())
        .def_readwrite("xyz", &urm::Pose::xyz)
        .def_readwrite("rpy", &urm::Pose::rpy)
        .def("__repr__", [](const urm::Pose& p) {
            return "<Pose xyz=[" + std::to_string(p.xyz[0]) + ", " +
                   std::to_string(p.xyz[1]) + ", " + std::to_string(p.xyz[2]) + "] rpy=[" +
                   std::to_string(p.rpy[0]) + ", " + std::to_string(p.rpy[1]) + ", " +
                   std::to_string(p.rpy[2]) + "]>";
        });

    // Mimic
    py::class_<urm::Mimic>(m, "Mimic")
        .def(py::init<>())
        .def_readwrite("joint",      &urm::Mimic::joint)
        .def_readwrite("multiplier", &urm::Mimic::multiplier)
        .def_readwrite("offset",     &urm::Mimic::offset)
        .def("__repr__", [](const urm::Mimic& mi) {
            return "<Mimic joint='" + mi.joint + "' mult=" +
                   std::to_string(mi.multiplier) + " offset=" +
                   std::to_string(mi.offset) + ">";
        });

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
        .def_readwrite("name",        &urm::Joint::name)
        .def_readwrite("type",        &urm::Joint::type)
        .def_readwrite("parent_link", &urm::Joint::parent_link)
        .def_readwrite("child_link",  &urm::Joint::child_link)
        .def_readwrite("kinematics",  &urm::Joint::kinematics)
        .def_readwrite("origin",      &urm::Joint::origin)
        .def_readwrite("axis",        &urm::Joint::axis)
        .def_readwrite("mimic",       &urm::Joint::mimic)
        .def_readwrite("dh",          &urm::Joint::dh)
        .def_readwrite("mdh",         &urm::Joint::mdh)
        .def_readwrite("limits",      &urm::Joint::limits)
        .def_readwrite("q",           &urm::Joint::q)
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
        .def_readwrite("schema_version", &urm::RobotModel::schema_version)
        .def_readwrite("convention",     &urm::RobotModel::convention)
        .def_readwrite("metadata",       &urm::RobotModel::metadata)
        .def_readwrite("base",           &urm::RobotModel::base)
        .def_readwrite("tool",           &urm::RobotModel::tool)
        .def_readwrite("tcp_link",       &urm::RobotModel::tcp_link)
        .def_readwrite("links",          &urm::RobotModel::links)
        .def_readwrite("joints",         &urm::RobotModel::joints)
        .def("dof",          &urm::RobotModel::dof,
             "Return the number of actuated (non-Fixed, non-mimic) DOF.")
        .def("q_index_map",  &urm::RobotModel::q_index_map,
             "Return mapping from joint index to DOF vector index (-1 for fixed/mimic joints).")
        .def("get_q",        &urm::RobotModel::get_q,
             "Return current joint values as a list of floats.")
        .def("set_q",        &urm::RobotModel::set_q, py::arg("q"),
             "Set joint values from a list of floats.")
        .def("validate",     &urm::RobotModel::validate,
             "Validate model consistency. Returns human-readable error messages.")
        .def("validate_physical", &urm::RobotModel::validate_physical,
             "Physical-data warnings (mass/COM/inertia). Empty list = OK.")
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
    m.def("default_limits", &urm::default_limits, py::arg("type"),
          "Return default JointLimits for a given JointType.");
    m.def("urm_to_json",    &urm::to_json,        py::arg("model"), py::arg("indent") = 2);
    m.def("urm_from_json",  &urm::from_json,      py::arg("json_str"));
}
