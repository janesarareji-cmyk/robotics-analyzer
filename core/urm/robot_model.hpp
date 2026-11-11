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

/// Metadata about the robot.
struct RobotMetadata {
    std::string name        = "Unnamed Robot";
    std::string description;
    std::string author;
    std::string version     = "1.0.0";
};

/// The Unified Robot Model — single source of truth.
/// Supports both serial chains and branching kinematic trees.
/// The `joints` list includes all joints (revolute, prismatic, fixed, mimic).
/// The `dof()` method counts non-fixed, non-mimic (actuated) joints only.
struct RobotModel {
    int                  schema_version = 2;               ///< Schema version (v2 from R3.1)
    RobotMetadata        metadata;
    DHConvention         convention     = DHConvention::Standard;
    Pose                 base{};                           ///< Base mount transform: world -> frame 0
    Pose                 tool{};                           ///< End-effector transform: tcp_link -> TCP
    int                  tcp_link       = -1;              ///< Target link for tool pose (-1 = last link)
    std::vector<Link>    links;
    std::vector<Joint>   joints;

    /// Return the number of actuated (non-Fixed, non-mimic) DOF.
    int dof() const;

    /// Return mapping from joint index to DOF vector index (-1 for fixed/mimic joints).
    std::vector<int> q_index_map() const;

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
