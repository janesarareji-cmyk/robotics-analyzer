#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <array>
#include <string>
#include <vector>
#include "kin/compiled_model.hpp"

namespace kin {

struct IKOptions {
    bool   position_only = false;
    std::array<double, 6> weights = {1, 1, 1, 1, 1, 1}; ///< rows: x y z rx ry rz
    double damping = 0.05;       ///< lambda in J^T (J J^T + lambda^2 I)^-1
    int    max_iter = 100;
    double tol_pos = 1e-6;       ///< m
    double tol_rot = 1e-6;       ///< rad
    double max_step = 0.3;       ///< rad or m per iteration, infinity norm
    bool   enforce_limits = true;
    double null_gain = 0.0;      ///< null-space joint-limit push, 0 = off
};

struct IKResult {
    std::vector<double> q;       ///< best-error iterate
    bool   converged = false;
    double pos_error = 0.0;      ///< unweighted norms at the returned q
    double rot_error = 0.0;
    int    iterations = 0;
    std::string message;         ///< "converged" | "max iterations" | "stalled"
};

/// Damped least squares IK for the TCP of the active chain (world target).
/// Convergence is tested on the weighted error rows (zero-weight rows ignored,
/// rotation rows ignored when position_only); reported errors are unweighted.
/// Throws std::invalid_argument on wrong q0 / limit vector sizes. Never throws
/// for unreachable targets.
IKResult solve_ik(const CompiledModel& cm, const std::vector<double>& q_limits_lo,
                  const std::vector<double>& q_limits_hi, const Eigen::Isometry3d& target,
                  const std::vector<double>& q0, const IKOptions& opt = IKOptions());

} // namespace kin
