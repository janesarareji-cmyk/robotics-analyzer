/**
 * bindings/kin_bindings.cpp
 * pybind11 bindings for forward kinematics (kin).
 * R2.5: Exposes fk(model), fk(model, q), and pose_to_iso(pose).
 */

#include <algorithm>
#include <array>
#include <cstring>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>

#include "kin/fk.hpp"
#include "kin/compiled_model.hpp"
#include "kin/compiled_arrays.hpp"
#include "kin/jacobian.hpp"
#include "kin/ik.hpp"
#include "kin/ik_multi.hpp"
#include "kin/singularity.hpp"
#include <stdexcept>
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

/// Copy a vector into a new numpy array of the given shape (copy, not view).
template <typename T>
static py::array_t<T> to_np(const std::vector<T>& v, std::vector<py::ssize_t> shape) {
    py::array_t<T> arr(shape);
    if (!v.empty()) {
        std::memcpy(arr.mutable_data(), v.data(), v.size() * sizeof(T));
    }
    return arr;
}

static py::dict compiled_arrays_to_dict(const kin::CompiledArrays& a) {
    const py::ssize_t n = a.n_joints;
    py::dict d;
    d["n_joints"] = a.n_joints;
    d["n_links"] = a.n_links;
    d["n_dof"] = a.n_dof;
    d["tcp_link"] = a.tcp_link;
    d["layout_version"] = a.layout_version;
    d["joint_index"] = to_np(a.joint_index, {n});
    d["parent_link"] = to_np(a.parent_link, {n});
    d["child_link"] = to_np(a.child_link, {n});
    d["type"] = to_np(a.type, {n});
    d["q_index"] = to_np(a.q_index, {n});
    d["src_q_index"] = to_np(a.src_q_index, {n});
    d["pre"] = to_np(a.pre, {n, 4, 4});
    d["post"] = to_np(a.post, {n, 4, 4});
    d["axis"] = to_np(a.axis, {n, 3});
    d["mult"] = to_np(a.mult, {n});
    d["offset"] = to_np(a.offset, {n});
    d["base"] = to_np(a.base, {4, 4});
    d["tool"] = to_np(a.tool, {4, 4});
    return d;
}

static kin::IKOptions parse_ik_options(const py::object& options) {

        kin::IKOptions o;

        if (!options.is_none()) {

            for (auto item : options.cast<py::dict>()) {

                const std::string k = py::str(item.first);

                py::handle v = item.second;

                if (k == "position_only") o.position_only = v.cast<bool>();

                else if (k == "weights") {

                    auto w = v.cast<std::vector<double>>();

                    if (w.size() != 6) throw std::invalid_argument("solve_ik: weights needs 6 values");

                    std::copy(w.begin(), w.end(), o.weights.begin());

                }

                else if (k == "damping") o.damping = v.cast<double>();

                else if (k == "max_iter") o.max_iter = v.cast<int>();

                else if (k == "tol_pos") o.tol_pos = v.cast<double>();

                else if (k == "tol_rot") o.tol_rot = v.cast<double>();

                else if (k == "max_step") o.max_step = v.cast<double>();

                else if (k == "enforce_limits") o.enforce_limits = v.cast<bool>();

                else if (k == "null_gain") o.null_gain = v.cast<double>();

                else throw std::invalid_argument("solve_ik: unknown option '" + k + "'");

            }

        }

    return o;

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

    m.def("jacobian", [](const kin::CompiledModel& cm, const std::vector<double>& q, int link,
                         const std::array<double, 3>& point_local) {
        return kin::jacobian(cm, q, link, Eigen::Vector3d(point_local[0], point_local[1], point_local[2]));
    }, py::arg("cm"), py::arg("q"), py::arg("link"),
       py::arg("point_local") = std::array<double, 3>{0.0, 0.0, 0.0},
       "World-frame 6 x n_dof Jacobian of a point fixed in link `link` (rows: v, w).");

    m.def("jacobian_tcp", [](const kin::CompiledModel& cm, const std::vector<double>& q) {
        return kin::jacobian_tcp(cm, q);
    }, py::arg("cm"), py::arg("q"),
       "World-frame 6 x n_dof Jacobian at the TCP (angular part of tcp_link).");

    m.def("solve_ik", [](const kin::CompiledModel& cm, const std::vector<double>& lo,
                         const std::vector<double>& hi, const Eigen::Matrix4d& target,
                         const std::vector<double>& q0, const py::object& options) {
        kin::IKOptions o = parse_ik_options(options);
        Eigen::Isometry3d T(target);
        kin::IKResult r = kin::solve_ik(cm, lo, hi, T, q0, o);
        py::dict d;
        d["q"] = r.q;
        d["converged"] = r.converged;
        d["pos_error"] = r.pos_error;
        d["rot_error"] = r.rot_error;
        d["iterations"] = r.iterations;
        d["message"] = r.message;
        return d;
    }, py::arg("cm"), py::arg("lo"), py::arg("hi"), py::arg("target"), py::arg("q0"),
       py::arg("options") = py::none(),
       "Damped least squares IK to a 4x4 world TCP target. Returns dict.");

    m.def("solve_ik_multi", [](const kin::CompiledModel& cm, const std::vector<double>& lo,
                               const std::vector<double>& hi, const Eigen::Matrix4d& target,
                               const std::vector<double>& q_current, const py::object& options,
                               int n_starts, uint64_t seed, double dedupe_tol) {
        kin::IKOptions o = parse_ik_options(options);
        Eigen::Isometry3d T(target);
        auto sols = kin::solve_ik_multi(cm, lo, hi, T, q_current, o, n_starts, seed, dedupe_tol);
        py::list out;
        for (const auto& s : sols) {
            py::dict d;
            d["q"] = s.q;
            d["pos_error"] = s.pos_error;
            d["rot_error"] = s.rot_error;
            d["distance"] = s.distance;
            out.append(d);
        }
        return out;
    }, py::arg("cm"), py::arg("lo"), py::arg("hi"), py::arg("target"), py::arg("q_current"),
       py::arg("options") = py::none(), py::arg("n_starts") = 30, py::arg("seed") = 42,
       py::arg("dedupe_tol") = 1e-3,
       "Multi-start IK: distinct validated solutions sorted by distance to q_current.");

    m.def("analyze_singularity", [](const Eigen::MatrixXd& J, const std::string& mode, double threshold) {
        const kin::SingularityInfo s = kin::analyze_singularity(J, mode, threshold);
        py::dict d;
        d["singular_values"] = s.singular_values;
        d["sigma_min"] = s.sigma_min;
        d["sigma_max"] = s.sigma_max;
        d["condition"] = s.condition;
        d["manipulability"] = s.manipulability;
        d["near_singular"] = s.near_singular;
        return d;
    }, py::arg("J"), py::arg("mode") = "full", py::arg("threshold") = 1e-3,
       "Singularity indicators of a 6 x n Jacobian. mode: full | position | angular. Returns dict.");

    m.def("compile_arrays", [](const urm::RobotModel& model) {
        return compiled_arrays_to_dict(kin::to_arrays(kin::compile(model)));
    }, py::arg("model"),
       "Flat frozen-layout arrays of the compiled model (see docs/compiled-model-layout.md).");

    m.def("effective_links", [](const urm::RobotModel& model) {
        return kin::effective_links(model);
    }, py::arg("model"),
       "Effective (parent_link, child_link) per URM joint (legacy 0/0 serial rule applied).");

    m.def("pose_to_iso", [](const urm::Pose& pose) {
        return Eigen::Matrix4d(kin::pose_to_iso(pose).matrix());
    }, py::arg("pose"),
       "Convert a 6D urm.Pose to a 4x4 homogeneous transformation matrix.");
}

