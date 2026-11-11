#pragma once
#include "joint.hpp"
#include "link.hpp"
#include <array>
#include <string>
#include <vector>

namespace urm {

/// Denavit-Hartenberg parameter convention for the robot.
enum class DHConvention {
    Standard,  ///< Classic Denavit-Hartenberg convention
    Modified,  ///< Modified Denavit-Hartenberg convention (Craig)
};

/// 6D Pose in Cartesian space: xyz position [m] and rpy rotation [rad].
/// Rotation is fixed-axis XYZ convention:
/// R = Rz(yaw) · Ry(pitch) · Rx(roll), with rpy = (roll, pitch, yaw).
struct Pose {
    std::array<double, 3> xyz{};
    std::array<double, 3> rpy{};
};

/// Metadata about the robot.
struct RobotMetadata {
    std::string name        = "Unnamed Robot";
    std::string description;
    std::string author;
    std::string version     = "1.0.0";
};

/// The Unified Robot Model — single source of truth.
/// Links and joints alternate: link[0] → joint[0] → link[1] → joint[1] → ...
/// The `joints` list includes all joints (revolute, prismatic, and fixed).
/// The `dof()` method counts non-fixed (actuated) joints only.
struct RobotModel {
    int                  schema_version = 1;
    RobotMetadata        metadata;
    DHConvention         convention     = DHConvention::Standard;
    Pose                 base{};  ///< Base mount transform: world -> frame 0
    Pose                 tool{};  ///< End-effector transform: last frame -> TCP
    std::vector<Link>    links;
    std::vector<Joint>   joints;

    /// Return the number of actuated (non-fixed) DOF.
    int dof() const;

    /// Return a joint value vector of length dof().
    std::vector<double> get_q() const;

    /// Set joint values from a vector of length dof().
    /// Throws std::invalid_argument if q.size() != dof().
    void set_q(const std::vector<double>& q);

    /// Validate model consistency. Returns human-readable error messages.
    /// Returns an empty list if the model is valid.
    std::vector<std::string> validate() const;
};

} // namespace urm
