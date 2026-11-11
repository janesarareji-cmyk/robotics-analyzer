#pragma once

#include <Eigen/Dense>
#include <vector>
#include "kin/compiled_model.hpp"

namespace kin {

/// Geometric Jacobian (6 x n_dof) of the point `point_local` (expressed in the
/// frame of link `link`) in the WORLD frame. Rows 0-2: linear velocity,
/// rows 3-5: angular velocity. Columns are per DOF; a mimic slave on the path
/// adds mult * its column to its master's DOF column. Joints not on the path
/// base -> link contribute zero. Throws std::invalid_argument if `link` is out
/// of range or q.size() != n_dof.
Eigen::MatrixXd jacobian(const CompiledModel& cm, const std::vector<double>& q,
                         int link, const Eigen::Vector3d& point_local = Eigen::Vector3d::Zero());

/// Jacobian at the TCP: link = tcp_link, point = translation of `tool`.
/// Angular part is that of link tcp_link (tool rotation does not matter).
Eigen::MatrixXd jacobian_tcp(const CompiledModel& cm, const std::vector<double>& q);

} // namespace kin
