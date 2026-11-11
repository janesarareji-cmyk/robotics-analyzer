#pragma once
#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif
#include <array>
#include <cmath>
#include <optional>
#include <string>

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

/// Kinematics parameterisation mode for a joint.
enum class JointKinematics {
    DH,          ///< Denavit-Hartenberg parameterisation (serial chains)
    OriginAxis,  ///< Rigid origin transform + 3D unit axis (URDF / tree kinematic trees)
};

/// Joint variable conventions:
/// - Revolute:  θ_eff = θ + q (joint position q adds to DH/MDH angle θ or rotates about axis)
/// - Prismatic: d_eff = d + q (joint displacement q adds to DH/MDH link offset d or translates along axis)
/// - Fixed:     q is ignored (rigid connection, zero DOF)
/// - Mimic:     q is ignored (derived from master joint: q_mimic = multiplier * q_master + offset)
/// Same convention applies to both standard DH, modified DH (MDH), and OriginAxis.

/// 6D Pose in Cartesian space: xyz position [m] and rpy rotation [rad].
/// Rotation is fixed-axis XYZ convention:
/// R = Rz(yaw) · Ry(pitch) · Rx(roll), with rpy = (roll, pitch, yaw).
struct Pose {
    std::array<double, 3> xyz{};
    std::array<double, 3> rpy{};
};

/// Standard Denavit–Hartenberg parameters for one joint.
struct DHParams {
    double a      = 0.0;  ///< Link length [m]
    double alpha  = 0.0;  ///< Link twist [rad]
    double d      = 0.0;  ///< Link offset [m]
    double theta  = 0.0;  ///< Joint angle [rad] (variable for revolute)
};

/// Modified DH parameters (Craig convention).
/// In Craig convention, parameters `a` and `alpha` stored on joint i correspond
/// to link length a_{i-1} and link twist α_{i-1} between axes i-1 and i.
struct MDHParams {
    double a      = 0.0;  ///< Preceding link length a_{i-1} [m]
    double alpha  = 0.0;  ///< Preceding link twist α_{i-1} [rad]
    double d      = 0.0;  ///< Link offset d_i along joint axis i [m]
    double theta  = 0.0;  ///< Joint angle θ_i about joint axis i [rad]
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

/// Mimic joint specification: q = multiplier * q_master + offset.
struct Mimic {
    std::string joint;            ///< Name of master joint
    double      multiplier = 1.0; ///< Multiplier coefficient
    double      offset     = 0.0; ///< Constant offset
};

/// One joint in the kinematic tree.
struct Joint {
    std::string           name;
    JointType             type        = JointType::Revolute;
    int                   parent_link = 0;                  ///< Index of parent link
    int                   child_link  = 0;                  ///< Index of child link
    JointKinematics       kinematics  = JointKinematics::DH;///< Kinematic parameterisation mode
    Pose                  origin;                           ///< Origin transform from parent link
    std::array<double, 3> axis        = {0.0, 0.0, 1.0};    ///< Joint axis unit vector in child frame
    std::optional<Mimic>  mimic;                            ///< Optional mimic dependency
    DHParams              dh;                               ///< Standard DH parameters
    MDHParams             mdh;                              ///< Modified DH parameters
    JointLimits           limits;
    double                q           = 0.0;                ///< Current joint value [rad or m]
};

} // namespace urm
