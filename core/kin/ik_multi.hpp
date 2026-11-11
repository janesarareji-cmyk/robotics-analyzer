#pragma once

#include <cstdint>
#include <vector>
#include "kin/ik.hpp"

namespace kin {

struct IKSolution {
    std::vector<double> q;
    double pos_error = 0.0;   ///< unweighted, recomputed with forward_kinematics
    double rot_error = 0.0;
    double distance = 0.0;    ///< inf-norm distance to q_current
};

/// Multi-start IK. Start 0 = q_current, starts 1..n_starts-1 uniform in
/// [lo, hi] (std::mt19937_64(seed)). Converged results are deduped (inf-norm
/// < dedupe_tol; revolute DOFs whose limits span >= 2*pi compare after
/// wrapping to (-pi, pi]), validated (limits tol 1e-9, FK error re-computed
/// against the weighted tolerances of `opt`) and sorted by distance to
/// q_current ascending. Throws std::invalid_argument on bad sizes / n_starts < 1.
std::vector<IKSolution> solve_ik_multi(const CompiledModel& cm, const std::vector<double>& lo,
                                       const std::vector<double>& hi,
                                       const Eigen::Isometry3d& target,
                                       const std::vector<double>& q_current,
                                       const IKOptions& opt = IKOptions(), int n_starts = 30,
                                       uint64_t seed = 42, double dedupe_tol = 1e-3);

} // namespace kin
