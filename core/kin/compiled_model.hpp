#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <utility>
#include <vector>
#include "urm/robot_model.hpp"

namespace kin {

/// Compiled joint. Every joint (DH, MDH, origin_axis) is turned into
///     A = pre * motion(axis, q_eff) * post
/// where motion is AngleAxis(q_eff, axis) (revolute), Translation(axis * q_eff)
/// (prismatic) or identity (fixed), and
///     q_eff = mult * q[src_q_index] + offset.
/// For DH joints the stored theta/d zero point is folded into `pre`
/// (Rz(theta0) resp. Tz(d0)), so mimic scaling never scales it.
struct CompiledJoint {
    int joint_index;          ///< index into RobotModel::joints
    int parent_link;          ///< effective parent link (see legacy rule)
    int child_link;           ///< effective child link
    urm::JointType type;
    Eigen::Isometry3d pre;
    Eigen::Isometry3d post;
    Eigen::Vector3d axis;     ///< unit vector, in the frame after `pre`
    int q_index;              ///< own dof index; -1 for fixed and mimic joints
    int src_q_index;          ///< dof index that drives this joint (-1 for fixed)
    double mult;              ///< q_eff = mult * q[src_q_index] + offset
    double offset;            ///< normal joint: mult = 1, offset = 0
};

/// Compiled kinematic model. Compile once, reuse for many q (R5 relies on it).
struct CompiledModel {
    std::vector<CompiledJoint> joints;   ///< sorted by child_link ascending
    int n_links = 0;
    int n_dof = 0;
    int tcp_link = 0;                    ///< resolved, never -1
    Eigen::Isometry3d base = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d tool = Eigen::Isometry3d::Identity();
};

/// Legacy serial test: true when joints.size() >= 1 AND every joint has
/// parent_link == 0 AND child_link == 0 (R2 serial data; never valid in
/// schema v2 because child_link 0 is forbidden). A mixed model is not legacy.
bool is_legacy_serial(const urm::RobotModel& model);

/// Effective (parent, child) per joint index. Pure: never throws on link count.
/// Legacy serial models (see is_legacy_serial) map joint i to (i, i + 1);
/// all other models return the stored parent_link / child_link.
/// The link-count check for legacy models lives in compile().
/// The model is never modified.
std::vector<std::pair<int, int>> effective_links(const urm::RobotModel& model);

/// Compile a model. Does not call validate(). Throws std::invalid_argument on:
/// link index out of range, parent >= child, duplicate child, missing mimic
/// target, mimic cycle, mimic chain ending on a fixed joint, legacy model with
/// links.size() != joints.size() + 1, zero axis. tcp_link == -1 -> last link.
CompiledModel compile(const urm::RobotModel& model);

/// URM joint indices on the path link 0 -> resolved tcp_link, root to tip
/// (fixed and mimic joints included). Throws std::invalid_argument if tcp_link
/// is out of range or not reachable.
std::vector<int> active_chain(const urm::RobotModel& model);

} // namespace kin
