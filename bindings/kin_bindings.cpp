/**
 * bindings/kin_bindings.cpp
 * pybind11 bindings for forward kinematics (kin).
 * R2.5: Exposes fk(model), fk(model, q), and pose_to_iso(pose).
 */

#include <algorithm>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>

#include "kin/fk.hpp"
#include "kin/compiled_model.hpp"
#include "urm/robot_model.hpp"

namespace py = pybind11;

static py::dict fk_result_to_dict(const kin::FKResult& res, const std::vector<int>& active = {}) {
    py::dict d;
    py::list frames_list;
    for (const auto& f : res.frames) {
        frames_list.append(Eigen::Matrix4d(f.matrix()));
    }
    d["frames"] = frames_list;
    d["tcp"] = Eigen::Matrix4d(res.tcp.matrix());
    d["active_joints"] = active;
    return d;
}

void bind_kin(py::module_& m) {
    py::class_<kin::CompiledModel>(m, "CompiledModel")
        .def_readonly("n_links", &kin::CompiledModel::n_links)
        .def_readonly("n_dof", &kin::CompiledModel::n_dof)
        .def_readonly("tcp_link", &kin::CompiledModel::tcp_link);

    m.def("fk", [](const urm::RobotModel& model) {
        return fk_result_to_dict(kin::forward_kinematics(model), kin::active_chain(model));
    }, py::arg("model"),
       "Compute forward kinematics using joint q values stored in model.");

    m.def("fk", [](const urm::RobotModel& model, const std::vector<double>& q) {
        return fk_result_to_dict(kin::forward_kinematics(model, q), kin::active_chain(model));
    }, py::arg("model"), py::arg("q"),
       "Compute forward kinematics for an explicit joint value vector q (size dof()).");

    m.def("active_chain", [](const urm::RobotModel& model) {
        return kin::active_chain(model);
    }, py::arg("model"),
       "URM joint indices on the path link 0 -> tcp_link, root to tip.");

    m.def("compile", [](const urm::RobotModel& model) {
        return kin::compile(model);
    }, py::arg("model"),
       "Compile a model once for repeated FK (raises ValueError on bad model).");

    m.def("fk_compiled", [](const kin::CompiledModel& cm, const std::vector<double>& q) {
        std::vector<int> active;
        int cur = cm.tcp_link;
        while (cur != 0) {
            int found = -1;
            for (const auto& j : cm.joints) if (j.child_link == cur) { found = j.joint_index; cur = j.parent_link; break; }
            if (found < 0) break;
            active.push_back(found);
        }
        std::reverse(active.begin(), active.end());
        return fk_result_to_dict(kin::forward_kinematics(cm, q), active);
    }, py::arg("cm"), py::arg("q"),
       "FK on a compiled model; same dict as fk(model, q).");

    m.def("pose_to_iso", [](const urm::Pose& pose) {
        return Eigen::Matrix4d(kin::pose_to_iso(pose).matrix());
    }, py::arg("pose"),
       "Convert a 6D urm.Pose to a 4x4 homogeneous transformation matrix.");
}

