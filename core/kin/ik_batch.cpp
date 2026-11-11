#include "kin/ik_batch.hpp"

#include <cmath>
#include <stdexcept>

#include "compute/parallel.hpp"

namespace kin {

namespace {
bool all_finite(const double* p, std::size_t k) {
    for (std::size_t i = 0; i < k; ++i)
        if (!std::isfinite(p[i])) return false;
    return true;
}
}  // namespace

IkBatchResult ik_batch_cpu(const CompiledModel& cm, const std::vector<double>& lo,
                           const std::vector<double>& hi, const double* targets, std::size_t n,
                           const double* q0, bool q0_broadcast, const IKOptions& opt,
                           int n_threads) {
    const std::size_t dof = static_cast<std::size_t>(cm.n_dof);
    if (lo.size() != dof || hi.size() != dof)
        throw std::invalid_argument("solve_ik_batch: limit vectors size != n_dof");
    IkBatchResult r;
    r.n = n;
    r.n_dof = cm.n_dof;
    r.q.assign(n * dof, 0.0);
    r.status.assign(n, IK_NAN);
    r.pos_error.assign(n, 0.0);
    r.rot_error.assign(n, 0.0);
    r.iterations.assign(n, 0);
    if (n == 0) return r;

    compute::parallel_for(n, n_threads, 32, [&](std::size_t b, std::size_t e) {
        std::vector<double> start(dof);
        for (std::size_t i = b; i < e; ++i) {
            const double* q0i = q0_broadcast ? q0 : q0 + i * dof;
            const double* ti = targets + i * 16;
            for (std::size_t d = 0; d < dof; ++d) {
                start[d] = q0i[d];
                r.q[i * dof + d] = q0i[d];
            }
            if (!all_finite(ti, 16) || !all_finite(q0i, dof)) {
                r.status[i] = IK_NAN;
                continue;
            }
            Eigen::Matrix4d M;
            for (int a = 0; a < 4; ++a)
                for (int c = 0; c < 4; ++c) M(a, c) = ti[a * 4 + c];
            const Eigen::Isometry3d T(M);
            const IKResult res = solve_ik(cm, lo, hi, T, start, opt);
            for (std::size_t d = 0; d < dof; ++d) r.q[i * dof + d] = res.q[d];
            r.pos_error[i] = res.pos_error;
            r.rot_error[i] = res.rot_error;
            r.iterations[i] = res.iterations;
            if (!std::isfinite(res.pos_error) || !std::isfinite(res.rot_error) ||
                !all_finite(res.q.data(), dof))
                r.status[i] = IK_NAN;
            else if (res.converged)
                r.status[i] = IK_CONVERGED;
            else if (res.message == "stalled")
                r.status[i] = IK_STALLED;
            else
                r.status[i] = IK_MAX_ITER;
        }
    });
    return r;
}

}  // namespace kin
