#include "kin/ik_multi.hpp"
#include "kin/fk.hpp"
#include <algorithm>
#include <cmath>
#include <random>
#include <stdexcept>

namespace kin {

namespace {

constexpr double kPi = 3.14159265358979323846;

double wrap_pi(double a) {
    // to (-pi, pi]
    double r = std::fmod(a + kPi, 2.0 * kPi);
    if (r <= 0.0) r += 2.0 * kPi;
    return r - kPi;
}

} // namespace

std::vector<IKSolution> solve_ik_multi(const CompiledModel& cm, const std::vector<double>& lo,
                                       const std::vector<double>& hi,
                                       const Eigen::Isometry3d& target,
                                       const std::vector<double>& q_current, const IKOptions& opt,
                                       int n_starts, uint64_t seed, double dedupe_tol) {
    const int n = cm.n_dof;
    if (static_cast<int>(q_current.size()) != n)
        throw std::invalid_argument("solve_ik_multi: q_current size != n_dof");
    if (static_cast<int>(lo.size()) != n || static_cast<int>(hi.size()) != n)
        throw std::invalid_argument("solve_ik_multi: limit vectors size != n_dof");
    if (n_starts < 1) throw std::invalid_argument("solve_ik_multi: n_starts < 1");
    for (int i = 0; i < n; ++i)
        if (!std::isfinite(lo[i]) || !std::isfinite(hi[i]))
            throw std::invalid_argument("solve_ik_multi: joint limits must be finite");

    // Per-DOF: wrap when revolute master and limits span >= 2*pi.
    std::vector<char> wrap(n, 0);
    for (const auto& j : cm.joints)
        if (j.q_index >= 0 && j.type == urm::JointType::Revolute &&
            hi[j.q_index] - lo[j.q_index] >= 2.0 * kPi)
            wrap[j.q_index] = 1;

    const int rows = opt.position_only ? 3 : 6;

    std::mt19937_64 rng(seed);
    std::vector<IKSolution> kept;

    auto same = [&](const std::vector<double>& a, const std::vector<double>& b) {
        double d = 0.0;
        for (int i = 0; i < n; ++i) {
            double diff = a[i] - b[i];
            if (wrap[i]) diff = wrap_pi(a[i]) - wrap_pi(b[i]), diff = std::abs(wrap_pi(diff));
            d = std::max(d, std::abs(diff));
        }
        return d < dedupe_tol;
    };

    for (int s = 0; s < n_starts; ++s) {
        std::vector<double> q0 = q_current;
        if (s > 0) {
            for (int i = 0; i < n; ++i) {
                if (hi[i] > lo[i]) {
                    std::uniform_real_distribution<double> u(lo[i], hi[i]);
                    q0[i] = u(rng);
                } else {
                    q0[i] = lo[i];
                }
            }
        }
        const IKResult r = solve_ik(cm, lo, hi, target, q0, opt);
        if (!r.converged) continue;

        // validation: limits
        bool ok = true;
        for (int i = 0; i < n; ++i)
            if (r.q[i] < lo[i] - 1e-9 || r.q[i] > hi[i] + 1e-9) { ok = false; break; }
        if (!ok) continue;

        // validation: recompute FK error (weighted rows, like the solver)
        const Eigen::Isometry3d T = forward_kinematics(cm, r.q).tcp;
        Eigen::Matrix<double, 6, 1> e;
        e.head<3>() = target.translation() - T.translation();
        Eigen::AngleAxisd aa(Eigen::Matrix3d(target.linear() * T.linear().transpose()));
        Eigen::Vector3d rv = aa.axis() * aa.angle();
        if (!rv.allFinite()) rv.setZero();
        e.tail<3>() = rv;
        Eigen::Matrix<double, 6, 1> w;
        for (int i = 0; i < 6; ++i) w[i] = opt.weights[i];
        const double wp = (e.head<3>().array() * w.head<3>().array()).matrix().norm();
        const double wr = (rows == 6) ? (e.tail<3>().array() * w.tail<3>().array()).matrix().norm() : 0.0;
        if (!(wp < opt.tol_pos) || !(wr < opt.tol_rot || rows == 3)) continue;

        bool dup = false;
        for (const auto& k : kept)
            if (same(k.q, r.q)) { dup = true; break; }
        if (dup) continue;

        IKSolution sol;
        sol.q = r.q;
        sol.pos_error = e.head<3>().norm();
        sol.rot_error = e.tail<3>().norm();
        double d = 0.0;
        for (int i = 0; i < n; ++i) d = std::max(d, std::abs(r.q[i] - q_current[i]));
        sol.distance = d;
        kept.push_back(std::move(sol));
    }
    std::stable_sort(kept.begin(), kept.end(),
                     [](const IKSolution& a, const IKSolution& b) { return a.distance < b.distance; });
    return kept;
}

} // namespace kin
