#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <vector>
#include "kin/compiled_model.hpp"
#include "urm/robot_model.hpp"

namespace dyn {

using Vec6 = Eigen::Matrix<double, 6, 1>;
using Mat6 = Eigen::Matrix<double, 6, 6>;

/// Joint friction (R4.7 fills the maths). Arrays are per DOF (size n_dof).
/// R4.6 only reserves the argument; no friction is applied yet.
struct Friction {
    std::vector<double> viscous;
    std::vector<double> coulomb;
    double eps = 1e-3;
};

/// Per-joint data of the dynamics model (same order as CompiledModel::joints).
struct DynJoint {
    int parent_link = 0;
    int child_link = 0;
    urm::JointType type = urm::JointType::Revolute;
    int src_q_index = -1;
    double mult = 1.0;
    double offset = 0.0;
    Eigen::Isometry3d pre = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d post = Eigen::Isometry3d::Identity();
    Eigen::Vector3d axis = Eigen::Vector3d::UnitZ();
    Vec6 S = Vec6::Zero();   ///< motion subspace in the LINK frame (after post); zero for fixed
};

/// Dynamics model, built once per robot. Spatial vectors are [angular; linear].
struct DynModel {
    int n_dof = 0;
    int n_links = 0;
    Eigen::Isometry3d base = Eigen::Isometry3d::Identity();
    std::vector<DynJoint> joints;   ///< sorted by child_link (as CompiledModel)
    std::vector<Mat6> I;            ///< spatial inertia per link at the link origin (size n_links)
};

/// Build the dynamics model. Does not call validate_physical(); mass 0 = zero inertia.
DynModel build_dyn_model(const urm::RobotModel& model, const kin::CompiledModel& cm);

/// Inverse dynamics (RNEA, Featherstone). q, qd, qdd have n_dof entries, else
/// std::invalid_argument. `gravity_world` is the gravity vector in the WORLD frame.
/// Result: actuator effort per DOF. `friction` is reserved (null: rigid body only).
std::vector<double> inverse_dynamics(const DynModel& dm, const std::vector<double>& q,
                                     const std::vector<double>& qd, const std::vector<double>& qdd,
                                     const Eigen::Vector3d& gravity_world,
                                     const Friction* friction = nullptr);

/// RNEA(q, 0, 0, g).
std::vector<double> gravity_torque(const DynModel& dm, const std::vector<double>& q,
                                   const Eigen::Vector3d& gravity_world);
/// RNEA(q, qd, 0, g) (includes gravity).
std::vector<double> bias_torque(const DynModel& dm, const std::vector<double>& q,
                                const std::vector<double>& qd, const Eigen::Vector3d& gravity_world);
/// bias - gravity = C(q, qd) qd.
std::vector<double> coriolis_torque(const DynModel& dm, const std::vector<double>& q,
                                    const std::vector<double>& qd, const Eigen::Vector3d& gravity_world);

/// Test hook: world frames of all links built from the dynamics transforms.
std::vector<Eigen::Isometry3d> dyn_frames(const DynModel& dm, const std::vector<double>& q);

/// Test hook: spatial kinetic energy 0.5 * sum_k v_k^T I_k v_k (link frames).
double kinetic_energy_spatial(const DynModel& dm, const std::vector<double>& q,
                              const std::vector<double>& qd);

} // namespace dyn
