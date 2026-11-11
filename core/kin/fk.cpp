#include "fk.hpp"
#include "compiled_model.hpp"
#include <stdexcept>
#include <string>

namespace kin {

Eigen::Isometry3d pose_to_iso(const urm::Pose& pose) {
    Eigen::Isometry3d iso = Eigen::Isometry3d::Identity();
    iso.translation() = Eigen::Vector3d(pose.xyz[0], pose.xyz[1], pose.xyz[2]);
    Eigen::AngleAxisd roll(pose.rpy[0], Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitch(pose.rpy[1], Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yaw(pose.rpy[2], Eigen::Vector3d::UnitZ());
    iso.linear() = (yaw * pitch * roll).toRotationMatrix();
    return iso;
}

Eigen::Isometry3d dh_transform(double a, double alpha, double d, double theta) {
    Eigen::Isometry3d A = Eigen::Isometry3d::Identity();
    Eigen::AngleAxisd rotZ(theta, Eigen::Vector3d::UnitZ());
    Eigen::Translation3d transZ(0.0, 0.0, d);
    Eigen::Translation3d transX(a, 0.0, 0.0);
    Eigen::AngleAxisd rotX(alpha, Eigen::Vector3d::UnitX());
    A = rotZ * transZ * transX * rotX;
    return A;
}

Eigen::Isometry3d mdh_transform(double a, double alpha, double d, double theta) {
    Eigen::Isometry3d A = Eigen::Isometry3d::Identity();
    Eigen::AngleAxisd rotX(alpha, Eigen::Vector3d::UnitX());
    Eigen::Translation3d transX(a, 0.0, 0.0);
    Eigen::AngleAxisd rotZ(theta, Eigen::Vector3d::UnitZ());
    Eigen::Translation3d transZ(0.0, 0.0, d);
    A = rotX * transX * rotZ * transZ;
    return A;
}

FKResult forward_kinematics(const CompiledModel& cm, const std::vector<double>& q) {
    if (static_cast<int>(q.size()) != cm.n_dof) {
        throw std::invalid_argument("q size mismatch: expected " +
                                    std::to_string(cm.n_dof) + ", got " +
                                    std::to_string(q.size()));
    }
    FKResult result;
    result.frames.assign(cm.n_links, Eigen::Isometry3d::Identity());
    result.frames[0] = cm.base;
    for (const auto& j : cm.joints) {
        Eigen::Isometry3d A = j.pre;
        if (j.type != urm::JointType::Fixed) {
            const double qe = j.mult * q[j.src_q_index] + j.offset;
            if (j.type == urm::JointType::Revolute) {
                A = A * Eigen::Isometry3d(Eigen::AngleAxisd(qe, j.axis));
            } else {
                A = A * Eigen::Isometry3d(Eigen::Translation3d(j.axis * qe));
            }
        }
        A = A * j.post;
        result.frames[j.child_link] = result.frames[j.parent_link] * A;
    }
    result.tcp = result.frames[cm.tcp_link] * cm.tool;
    return result;
}

FKResult forward_kinematics(const urm::RobotModel& model) {
    return forward_kinematics(compile(model), model.get_q());
}

FKResult forward_kinematics(const urm::RobotModel& model, const std::vector<double>& q) {
    return forward_kinematics(compile(model), q);
}

} // namespace kin

