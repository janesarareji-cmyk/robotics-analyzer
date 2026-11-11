#pragma once

#include <cstddef>
#include <vector>
#include "kin/compiled_model.hpp"

namespace kin {

/// Batch FK result in the BASE frame (the model base transform is NOT applied;
/// the tool transform is). Structure of arrays: position [3][n], rotation [9][n]
/// (row-major 3x3 per sample: r00 r01 r02 r10 ... r22).
struct FkBatchResult {
    std::size_t n = 0;
    std::vector<double> position;   ///< 3 * n, index c * n + i
    std::vector<double> rotation;   ///< 9 * n, index e * n + i
};

/// Indices into cm.joints of the path link 0 -> cm.tcp_link, root to tip
/// (fixed and mimic joints included). Throws std::invalid_argument when the
/// tcp link is not reachable from link 0.
std::vector<int> active_path(const CompiledModel& cm);

/// Multi-threaded double FK for n samples. q is SoA [n_dof][n] with leading
/// dimension ld >= n (element (d, i) at q[d * ld + i]). Only the active path is
/// walked. n_threads = 0 uses hardware_concurrency.
/// Throws std::invalid_argument if ld < n (and n_dof > 0).
FkBatchResult fk_batch_cpu(const CompiledModel& cm, const double* q, std::size_t n,
                           std::size_t ld, int n_threads = 0);

}  // namespace kin
