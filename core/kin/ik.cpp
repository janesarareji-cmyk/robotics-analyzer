#include "kin/ik.hpp"
#include "kin/fk.hpp"
#include "kin/jacobian.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace kin {

namespace {

struct Err {
    Eigen::Matrix<double, 6, 1> e = Eigen::Matrix<double, 6, 1>::Zero();  // unweighted
    double pos = 0.0, rot = 0.0;                                           // unweighted norms
};

Err pose_error(const CompiledModel& cm, const std::vector<double>& q,
               const Eigen::Isometry3d& target) {
    const Eigen::Isometry3d T = forward_kinematics(cm, q).tcp;
    Err r;
    r.e.head<3>() = target.translation() - T.translation();
    const Eigen::Matrix3d Rerr = target.linear() * T.linear().transpose();
    Eigen::AngleAxisd aa(Rerr);  // angle in [0, pi]
    Eigen::Vector3d rv = aa.axis() * aa.angle();
    if (!rv.allFinite()) rv.setZero();
    r.e.tail<3>() = rv;
    r.pos = r.e.head<3>().norm();
    r.rot = r.e.tail<3>().norm();
    return r;
}

} // namespace

IKResult solve_ik(const CompiledModel& cm, const std::vector<double>& lo,
                  const std::vector<double>& hi, const Eigen::Isometry3d& target,
                  const std::vector<double>& q0, const IKOptions& opt) {
    const int n = cm.n_dof;
    if (static_cast<int>(q0.size()) != n) throw std::invalid_argument("solve_ik: q0 size != n_dof");
    if (static_cast<int>(lo.size()) != n || static_cast<int>(hi.size()) != n)
        throw std::invalid_argument("solve_ik: limit vectors size != n_dof");

    const int rows = opt.position_only ? 3 : 6;
    Eigen::VectorXd w(rows);
    for (int i = 0; i < rows; ++i) w[i] = opt.weights[i];

    auto clamp_q = [&](std::vector<double>& q) {
        if (!opt.enforce_limits) return;
        for (int i = 0; i < n; ++i) q[i] = std::min(std::max(q[i], lo[i]), hi[i]);
    };
    auto weighted_norms = [&](const Err& er, double& wp, double& wr) {
        wp = (er.e.head<3>().array() * w.head<3>().array()).matrix().norm();
        wr = 0.0;
        if (rows == 6) wr = (er.e.tail<3>().array() * w.tail<3>().array()).matrix().norm();
    };
    auto score = [&](const Err& er) {
        double wp, wr;
        weighted_norms(er, wp, wr);
        return std::sqrt(wp * wp + wr * wr);
    };

    IKResult res;
    std::vector<double> q = q0;
    clamp_q(q);
    res.q = q;
    Err best_err = pose_error(cm, q, target);
    double best_score = score(best_err);
    res.message = "max iterations";

    int stall = 0;
    for (int it = 0;; ++it) {
        const Err er = pose_error(cm, q, target);
        const double sc = score(er);
        if (sc < best_score) { best_score = sc; best_err = er; res.q = q; }
        double wp, wr;
        weighted_norms(er, wp, wr);
        if (wp < opt.tol_pos && (rows == 3 || wr < opt.tol_rot)) {
            res.converged = true;
            res.message = "converged";
            res.iterations = it;
            break;
        }
        if (it >= opt.max_iter) { res.iterations = it; break; }
        if (n == 0) { res.message = "stalled"; res.iterations = it; break; }

        const Eigen::MatrixXd J = jacobian_tcp(cm, q).topRows(rows);
        const Eigen::MatrixXd Jw = w.asDiagonal() * J;
        const Eigen::VectorXd ew = w.asDiagonal() * er.e.head(rows);

        // Adaptive damping: full `damping` while the error is large, shrinking
        // linearly with the error below 0.1 (floor 1e-3 * damping) so the final
        // convergence is fast near singular poses.
        const double lam = opt.damping * std::max(1e-3, std::min(1.0, sc / 0.1));
        const double lam2 = lam * lam;
        Eigen::MatrixXd A = Jw * Jw.transpose();
        A.diagonal().array() += lam2;
        Eigen::LDLT<Eigen::MatrixXd> ldlt(A);
        Eigen::VectorXd dq = Jw.transpose() * ldlt.solve(ew);

        if (opt.null_gain > 0.0) {
            const Eigen::MatrixXd X = ldlt.solve(Jw);  // (J J^T + l^2 I)^-1 J
            const Eigen::MatrixXd P = Eigen::MatrixXd::Identity(n, n) - Jw.transpose() * X;
            Eigen::VectorXd push(n);
            for (int i = 0; i < n; ++i) {
                const double range = hi[i] - lo[i];
                const double mid = 0.5 * (hi[i] + lo[i]);
                push[i] = (range > 1e-12 && std::isfinite(range))
                              ? opt.null_gain * (mid - q[i]) / range : 0.0;
            }
            dq += P * push;
        }

        const double mx = dq.cwiseAbs().maxCoeff();
        if (mx > opt.max_step) dq *= opt.max_step / mx;

        std::vector<double> qn = q;
        for (int i = 0; i < n; ++i) qn[i] += dq[i];
        clamp_q(qn);
        double step = 0.0;
        for (int i = 0; i < n; ++i) step = std::max(step, std::abs(qn[i] - q[i]));
        q = qn;
        stall = (step < 1e-12) ? stall + 1 : 0;
        if (stall >= 2) {
            res.message = "stalled";
            res.iterations = it + 1;
            const Err ef = pose_error(cm, q, target);
            if (score(ef) < best_score) { best_score = score(ef); best_err = ef; res.q = q; }
            break;
        }
    }
    res.pos_error = best_err.pos;
    res.rot_error = best_err.rot;
    return res;
}

} // namespace kin
