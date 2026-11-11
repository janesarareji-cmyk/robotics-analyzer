#pragma once
#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif
#include <string>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace urm {

/// Joint type enumeration.
enum class JointType {
    Revolute,    ///< Rotation about joint axis (most common)
    Prismatic,   ///< Translation along joint axis
    Fixed,       ///< No DOF — rigid connection
};

/// Joint variable conventions:
/// - Revolute:  θ_eff = θ + q (joint position q adds to DH/MDH angle θ)
/// - Prismatic: d_eff = d + q (joint displacement q adds to DH/MDH link offset d)
/// - Fixed:     q is ignored (rigid connection, zero DOF)
/// Same convention applies to both standard DH and modified DH (MDH).

/// Standard Denavit–Hartenberg parameters for one joint.
struct DHParams {
    double a      = 0.0;  ///< Link length [m]
    double alpha  = 0.0;  ///< Link twist [rad]
    double d      = 0.0;  ///< Link offset [m]
    double theta  = 0.0;  ///< Joint angle [rad] (variable for revolute)
};

/// Modified DH parameters (Craig convention).
struct MDHParams {
    double a      = 0.0;  ///< Link length [m]
    double alpha  = 0.0;  ///< Link twist [rad]
    double d      = 0.0;  ///< Link offset [m]
    double theta  = 0.0;  ///< Joint angle [rad]
};

/// Joint limits.
struct JointLimits {
    double lower  = -M_PI; ///< Lower position limit [rad or m]
    double upper  =  M_PI; ///< Upper position limit [rad or m]
    double vel    = 1.0;   ///< Max velocity [rad/s or m/s]
    double effort = 100.0; ///< Max effort [Nm or N]
};

/// Return default joint limits based on joint type.
/// - Revolute:  [-π, π] rad
/// - Prismatic: [0, 0.5] m
/// - Fixed:     [0, 0]
JointLimits default_limits(JointType type);

/// One joint in the kinematic chain.
struct Joint {
    std::string name;
    JointType   type   = JointType::Revolute;
    DHParams    dh;      ///< Standard DH parameters
    MDHParams   mdh;     ///< Modified DH parameters
    JointLimits limits;
    double      q      = 0.0;  ///< Current joint value [rad or m]
};

} // namespace urm
