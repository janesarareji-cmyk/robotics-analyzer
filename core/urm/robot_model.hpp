#pragma once
#include "joint.hpp"
#include "link.hpp"
#include <string>
#include <vector>

namespace urm {

/// Metadata about the robot.
struct RobotMetadata {
    std::string name        = "Unnamed Robot";
    std::string description;
    std::string author;
    std::string version     = "1.0.0";
};

/// The Unified Robot Model — single source of truth.
/// Links and joints alternate: link[0] → joint[0] → link[1] → joint[1] → ...
/// Number of joints == number of DOF (excluding fixed joints).
struct RobotModel {
    RobotMetadata        metadata;
    std::vector<Link>    links;
    std::vector<Joint>   joints;

    /// Return the number of actuated (non-fixed) DOF.
    int dof() const;

    /// Return a joint value vector of length dof().
    std::vector<double> get_q() const;

    /// Set joint values from a vector of length dof().
    void set_q(const std::vector<double>& q);
};

} // namespace urm
