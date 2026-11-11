#include "fk.hpp"
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

static Eigen::Isometry3d compute_joint_transform(const urm::Joint& j, urm::DHConvention convention, double q_val) {
    double theta_eff = 0.0;
    double d_eff = 0.0;

    if (convention == urm::DHConvention::Standard) {
        theta_eff = j.dh.theta;
        d_eff = j.dh.d;
        if (j.type == urm::JointType::Revolute) {
            theta_eff += q_val;
        } else if (j.type == urm::JointType::Prismatic) {
            d_eff += q_val;
        }
        // Fixed joint: q is ignored
        return dh_transform(j.dh.a, j.dh.alpha, d_eff, theta_eff);
    } else {
        theta_eff = j.mdh.theta;
        d_eff = j.mdh.d;
        if (j.type == urm::JointType::Revolute) {
            theta_eff += q_val;
        } else if (j.type == urm::JointType::Prismatic) {
            d_eff += q_val;
        }
        // Fixed joint: q is ignored
        return mdh_transform(j.mdh.a, j.mdh.alpha, d_eff, theta_eff);
    }
}

FKResult forward_kinematics(const urm::RobotModel& model) {
    FKResult result;
    result.frames.reserve(model.joints.size() + 1);

    Eigen::Isometry3d current_frame = pose_to_iso(model.base);
    result.frames.push_back(current_frame);

    for (const auto& j : model.joints) {
        Eigen::Isometry3d A = compute_joint_transform(j, model.convention, j.q);
        current_frame = current_frame * A;
        result.frames.push_back(current_frame);
    }

    Eigen::Isometry3d tool_iso = pose_to_iso(model.tool);
    result.tcp = result.frames.back() * tool_iso;

    return result;
}

FKResult forward_kinematics(const urm::RobotModel& model, const std::vector<double>& q) {
    int expected_dof = model.dof();
    if (static_cast<int>(q.size()) != expected_dof) {
        throw std::invalid_argument("q size mismatch: expected " +
                                    std::to_string(expected_dof) + ", got " +
                                    std::to_string(q.size()));
    }

    FKResult result;
    result.frames.reserve(model.joints.size() + 1);

    Eigen::Isometry3d current_frame = pose_to_iso(model.base);
    result.frames.push_back(current_frame);

    size_t q_idx = 0;
    for (const auto& j : model.joints) {
        double q_val = 0.0;
        if (j.type != urm::JointType::Fixed) {
            q_val = q[q_idx++];
        }
        Eigen::Isometry3d A = compute_joint_transform(j, model.convention, q_val);
        current_frame = current_frame * A;
        result.frames.push_back(current_frame);
    }

    Eigen::Isometry3d tool_iso = pose_to_iso(model.tool);
    result.tcp = result.frames.back() * tool_iso;

    return result;
}

} // namespace kin
