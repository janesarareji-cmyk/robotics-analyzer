#include "compiled_arrays.hpp"
#include <cmath>
#include <stdexcept>
#include <string>

namespace kin {

namespace {

void put_iso(std::vector<double>& out, size_t k, const Eigen::Isometry3d& T) {
    const Eigen::Matrix4d M = T.matrix();
    for (int r = 0; r < 4; ++r)
        for (int c = 0; c < 4; ++c)
            out[k * 16 + static_cast<size_t>(r * 4 + c)] = M(r, c);
}

Eigen::Isometry3d get_iso(const std::vector<double>& in, size_t k) {
    Eigen::Matrix4d M;
    for (int r = 0; r < 4; ++r)
        for (int c = 0; c < 4; ++c)
            M(r, c) = in[k * 16 + static_cast<size_t>(r * 4 + c)];
    Eigen::Isometry3d T;
    T.matrix() = M;
    return T;
}

void put_iso16(std::vector<double>& out, const Eigen::Isometry3d& T) {
    out.assign(16, 0.0);
    put_iso(out, 0, T);
}

Eigen::Isometry3d get_iso16(const std::vector<double>& in, const char* name) {
    if (in.size() != 16) {
        throw std::invalid_argument(std::string("CompiledArrays: ") + name + " must have 16 values");
    }
    return get_iso(in, 0);
}

void check_size(size_t got, size_t want, const char* name) {
    if (got != want) {
        throw std::invalid_argument(std::string("CompiledArrays: wrong size for ") + name +
                                    " (got " + std::to_string(got) + ", want " +
                                    std::to_string(want) + ")");
    }
}

std::string at(const char* field, size_t k) {
    return std::string("CompiledArrays: ") + field + "[" + std::to_string(k) + "]";
}

void check_finite(const std::vector<double>& v, size_t from, size_t count, const std::string& what) {
    for (size_t i = from; i < from + count; ++i) {
        if (!std::isfinite(v[i])) throw std::invalid_argument(what + " is not finite");
    }
}

} // namespace

CompiledArrays to_arrays(const CompiledModel& cm) {
    const size_t n = cm.joints.size();
    CompiledArrays a;
    a.n_joints = static_cast<int32_t>(n);
    a.n_links = cm.n_links;
    a.n_dof = cm.n_dof;
    a.tcp_link = cm.tcp_link;
    a.layout_version = kCompiledLayoutVersion;

    a.joint_index.reserve(n);
    a.parent_link.reserve(n);
    a.child_link.reserve(n);
    a.type.reserve(n);
    a.q_index.reserve(n);
    a.src_q_index.reserve(n);
    a.pre.assign(n * 16, 0.0);
    a.post.assign(n * 16, 0.0);
    a.axis.assign(n * 3, 0.0);
    a.mult.reserve(n);
    a.offset.reserve(n);

    for (size_t k = 0; k < n; ++k) {
        const CompiledJoint& j = cm.joints[k];
        a.joint_index.push_back(j.joint_index);
        a.parent_link.push_back(j.parent_link);
        a.child_link.push_back(j.child_link);
        a.type.push_back(static_cast<int32_t>(j.type));
        a.q_index.push_back(j.q_index);
        a.src_q_index.push_back(j.src_q_index);
        put_iso(a.pre, k, j.pre);
        put_iso(a.post, k, j.post);
        a.axis[k * 3 + 0] = j.axis[0];
        a.axis[k * 3 + 1] = j.axis[1];
        a.axis[k * 3 + 2] = j.axis[2];
        a.mult.push_back(j.mult);
        a.offset.push_back(j.offset);
    }

    put_iso16(a.base, cm.base);
    put_iso16(a.tool, cm.tool);
    return a;
}

CompiledModel from_arrays(const CompiledArrays& a) {
    if (a.layout_version != kCompiledLayoutVersion) {
        throw std::invalid_argument("CompiledArrays: unsupported layout_version " +
                                    std::to_string(a.layout_version));
    }
    if (a.n_joints < 0) {
        throw std::invalid_argument("CompiledArrays: n_joints must not be negative");
    }
    const size_t n = static_cast<size_t>(a.n_joints);
    check_size(a.joint_index.size(), n, "joint_index");
    check_size(a.parent_link.size(), n, "parent_link");
    check_size(a.child_link.size(), n, "child_link");
    check_size(a.type.size(), n, "type");
    check_size(a.q_index.size(), n, "q_index");
    check_size(a.src_q_index.size(), n, "src_q_index");
    check_size(a.pre.size(), n * 16, "pre");
    check_size(a.post.size(), n * 16, "post");
    check_size(a.axis.size(), n * 3, "axis");
    check_size(a.mult.size(), n, "mult");
    check_size(a.offset.size(), n, "offset");

    // A1.4 (audit M5): range checks, the GPU path indexes with these values.
    if (a.n_links < 1) throw std::invalid_argument("CompiledArrays: n_links must be >= 1");
    if (a.n_dof < 0) throw std::invalid_argument("CompiledArrays: n_dof must not be negative");
    if (a.tcp_link < 0 || a.tcp_link >= a.n_links)
        throw std::invalid_argument("CompiledArrays: tcp_link out of range");
    check_finite(a.base, 0, a.base.size(), "CompiledArrays: base");
    check_finite(a.tool, 0, a.tool.size(), "CompiledArrays: tool");
    for (size_t k = 0; k < n; ++k) {
        const int32_t p = a.parent_link[k], c = a.child_link[k];
        if (p < 0 || p >= a.n_links) throw std::invalid_argument(at("parent_link", k) + " out of range");
        if (c < 0 || c >= a.n_links) throw std::invalid_argument(at("child_link", k) + " out of range");
        if (p >= c) throw std::invalid_argument(at("parent_link", k) + " must be less than child_link");
        if (a.type[k] < 0 || a.type[k] > 2)
            throw std::invalid_argument("CompiledArrays: unknown joint type value " + std::to_string(a.type[k]));
        const int32_t s = a.src_q_index[k];
        if (a.type[k] == 2) {
            if (s != -1) throw std::invalid_argument(at("src_q_index", k) + " must be -1 for a fixed joint");
        } else if (s < 0 || s >= a.n_dof) {
            throw std::invalid_argument(at("src_q_index", k) + " out of range");
        }
        check_finite(a.pre, k * 16, 16, at("pre", k));
        check_finite(a.post, k * 16, 16, at("post", k));
        check_finite(a.axis, k * 3, 3, at("axis", k));
        check_finite(a.mult, k, 1, at("mult", k));
        check_finite(a.offset, k, 1, at("offset", k));
    }

    CompiledModel cm;
    cm.n_links = a.n_links;
    cm.n_dof = a.n_dof;
    cm.tcp_link = a.tcp_link;
    cm.base = get_iso16(a.base, "base");
    cm.tool = get_iso16(a.tool, "tool");

    cm.joints.reserve(n);
    for (size_t k = 0; k < n; ++k) {
        if (a.type[k] < 0 || a.type[k] > 2) {
            throw std::invalid_argument("CompiledArrays: unknown joint type value " +
                                        std::to_string(a.type[k]));
        }
        CompiledJoint j;
        j.joint_index = a.joint_index[k];
        j.parent_link = a.parent_link[k];
        j.child_link = a.child_link[k];
        j.type = static_cast<urm::JointType>(a.type[k]);
        j.pre = get_iso(a.pre, k);
        j.post = get_iso(a.post, k);
        j.axis = Eigen::Vector3d(a.axis[k * 3 + 0], a.axis[k * 3 + 1], a.axis[k * 3 + 2]);
        j.q_index = a.q_index[k];
        j.src_q_index = a.src_q_index[k];
        j.mult = a.mult[k];
        j.offset = a.offset[k];
        cm.joints.push_back(j);
    }
    return cm;
}

} // namespace kin
