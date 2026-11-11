#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <vector>
#include "urm/robot_model.hpp"

namespace kin {

/// Convert a 6D urm::Pose (xyz position, rpy rotation) into an Eigen::Isometry3d.
/// Rotation follows fixed-axis XYZ convention:
/// R = Rz(yaw) · Ry(pitch) · Rx(roll), where rpy = (roll, pitch, yaw).
Eigen::Isometry3d pose_to_iso(const urm::Pose& pose);

/// Standard Denavit-Hartenberg transformation from frame i-1 to frame i:
/// A = Rz(θ) · Tz(d) · Tx(a) · Rx(α)
Eigen::Isometry3d dh_transform(double a, double alpha, double d, double theta);

/// Modified Denavit-Hartenberg (Craig convention) transformation from frame i-1 to frame i:
/// A = Rx(α) · Tx(a) · Rz(θ) · Tz(d)
/// In this convention, `a` and `alpha` stored on joint i represent Craig's preceding
/// link length a_{i-1} and link twist α_{i-1} between axes i-1 and i.
Eigen::Isometry3d mdh_transform(double a, double alpha, double d, double theta);

/// Result of forward kinematics computation.
/// - `frames`: `frames.size() == links.size()`; `frames[k]` = pose of link k in world.
///             `frames[0] = pose_to_iso(model.base)`.
/// - `tcp`: `tcp = frames[tcp_link] * pose_to_iso(model.tool)`.
struct FKResult {
    std::vector<Eigen::Isometry3d> frames;
    Eigen::Isometry3d              tcp;
};

struct CompiledModel;

/// FK on a compiled model. Callers compile once and reuse for many q.
/// q.size() must equal cm.n_dof, else std::invalid_argument.
/// Only allocates the result vector.
FKResult forward_kinematics(const CompiledModel& cm, const std::vector<double>& q);

/// Compute forward kinematics using joint values stored in model.joints[i].q.
FKResult forward_kinematics(const urm::RobotModel& model);

/// Compute forward kinematics for an explicit joint value vector `q`.
/// `q` corresponds to actuated (non-fixed) joints in sequential order.
/// Throws std::invalid_argument if q.size() != model.dof().
/// Does NOT modify the input model.
FKResult forward_kinematics(const urm::RobotModel& model, const std::vector<double>& q);

} // namespace kin

