/**
 * ik_batch.hpp
 * R5.4: multi-threaded batch IK on the CPU (double). Each item is one call of
 * kin::solve_ik, so results equal N separate solve_ik calls exactly.
 */
#pragma once

#include <cstddef>
#include <vector>

#include "kin/compiled_model.hpp"
#include "kin/ik.hpp"

namespace kin {

/// Per-item IK status (values are part of the Python API: cb.IK_*).
enum IkStatus : int {
    IK_CONVERGED = 0,
    IK_MAX_ITER = 1,
    IK_STALLED = 2,
    IK_NAN = 3,   ///< non-finite target / q0, or non-finite result
};

struct IkBatchResult {
    std::size_t n = 0;
    int n_dof = 0;
    std::vector<double> q;          ///< n * n_dof, row-major (item i at i * n_dof)
    std::vector<int> status;        ///< n, IkStatus
    std::vector<double> pos_error;  ///< n
    std::vector<double> rot_error;  ///< n
    std::vector<int> iterations;    ///< n
};

/// targets: n row-major 4x4 matrices (16 doubles each). q0: n * n_dof doubles, or
/// n_dof doubles when q0_broadcast (one start for all items). n_threads = 0 uses
/// hardware_concurrency; static chunking. Throws std::invalid_argument on wrong
/// lo/hi sizes. Non-finite target or q0 gives status IK_NAN (q = q0, no solve).
IkBatchResult ik_batch_cpu(const CompiledModel& cm, const std::vector<double>& lo,
                           const std::vector<double>& hi, const double* targets, std::size_t n,
                           const double* q0, bool q0_broadcast, const IKOptions& opt,
                           int n_threads = 0);

}  // namespace kin
