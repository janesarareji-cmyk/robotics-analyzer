/**
 * bindings/kin_bindings.cpp
 * pybind11 bindings for forward kinematics (kin).
 * R2.5: Exposes fk(model), fk(model, q), and pose_to_iso(pose).
 */

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>

#include "kin/fk.hpp"
#include "urm/robot_model.hpp"

namespace py = pybind11;

static py::dict fk_result_to_dict(const kin::FKResult& res) {
    py::dict d;
    py::list frames_list;
    for (const auto& f : res.frames) {
        frames_list.append(Eigen::Matrix4d(f.matrix()));
    }
    d["frames"] = frames_list;
    d["tcp"] = Eigen::Matrix4d(res.tcp.matrix());
    return d;
}

void bind_kin(py::module_& m) {
    m.def("fk", [](const urm::RobotModel& model) {
        return fk_result_to_dict(kin::forward_kinematics(model));
    }, py::arg("model"),
       "Compute forward kinematics using joint q values stored in model.");

    m.def("fk", [](const urm::RobotModel& model, const std::vector<double>& q) {
        return fk_result_to_dict(kin::forward_kinematics(model, q));
    }, py::arg("model"), py::arg("q"),
       "Compute forward kinematics for an explicit joint value vector q.");

    m.def("pose_to_iso", [](const urm::Pose& pose) {
        return Eigen::Matrix4d(kin::pose_to_iso(pose).matrix());
    }, py::arg("pose"),
       "Convert a 6D urm.Pose to a 4x4 homogeneous transformation matrix.");
}
