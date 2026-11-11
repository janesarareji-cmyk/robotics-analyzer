#include "dyn/dynamics.hpp"

#include <cmath>
#include <stdexcept>
#include <string>

namespace dyn {

using Eigen::Matrix3d;
using Eigen::Vector3d;
using urm::JointType;

namespace {

Matrix3d skew(const Vector3d& p) {
    Matrix3d S;
    S << 0, -p.z(), p.y(),
         p.z(), 0, -p.x(),
         -p.y(), p.x(), 0;
    return S;
}

/// ^kX_lambda from A = ^lambda T_k (frame k -> frame lambda): [R^T 0; -R^T[p] R^T].
Mat6 motion_xform_inv(const Eigen::Isometry3d& A) {
    const Matrix3d R = A.linear();
    const Vector3d p = A.translation();
    Mat6 X = Mat6::Zero();
    X.topLeftCorner<3, 3>() = R.transpose();
    X.bottomRightCorner<3, 3>() = R.transpose();
    X.bottomLeftCorner<3, 3>() = -R.transpose() * skew(p);
    return X;
}

/// v x (motion cross product): [[w] 0; [v] [w]].
Mat6 crossm(const Vec6& v) {
    Mat6 C = Mat6::Zero();
    const Matrix3d W = skew(v.head<3>());
    C.topLeftCorner<3, 3>() = W;
    C.bottomRightCorner<3, 3>() = W;
    C.bottomLeftCorner<3, 3>() = skew(v.tail<3>());
    return C;
}

/// v x* (force cross product): -(v x)^T.
Mat6 crossf(const Vec6& v) { return -crossm(v).transpose(); }

Eigen::Isometry3d joint_transform(const DynJoint& j, const std::vector<double>& q) {
    Eigen::Isometry3d A = j.pre;
    if (j.type == JointType::Revolute) {
        const double qe = j.mult * q[j.src_q_index] + j.offset;
        A = A * Eigen::Isometry3d(Eigen::AngleAxisd(qe, j.axis));
    } else if (j.type == JointType::Prismatic) {
        const double qe = j.mult * q[j.src_q_index] + j.offset;
        A = A * Eigen::Isometry3d(Eigen::Translation3d(j.axis * qe));
    }
    return A * j.post;
}

void check_size(const DynModel& dm, const std::vector<double>& v, const char* name) {
    if (static_cast<int>(v.size()) != dm.n_dof)
        throw std::invalid_argument(std::string("dynamics: ") + name + " needs n_dof entries");
}

struct Kin {
    std::vector<Mat6> X;   ///< X[k] = ^kX_lambda (per link, index 0 unused)
    std::vector<Vec6> v, a;
};

/// Forward pass of RNEA.
Kin forward_pass(const DynModel& dm, const std::vector<double>& q, const std::vector<double>& qd,
                 const std::vector<double>& qdd, const Vector3d& g) {
    Kin k;
    k.X.assign(dm.n_links, Mat6::Identity());
    k.v.assign(dm.n_links, Vec6::Zero());
    k.a.assign(dm.n_links, Vec6::Zero());
    const Matrix3d Rb = dm.base.linear();
    k.a[0].tail<3>() = -(Rb.transpose() * g);
    for (const DynJoint& j : dm.joints) {
        const int c = j.child_link, p = j.parent_link;
        k.X[c] = motion_xform_inv(joint_transform(j, q));
        double qde = 0.0, qdde = 0.0;
        if (j.type != JointType::Fixed) {
            qde = j.mult * qd[j.src_q_index];
            qdde = j.mult * qdd[j.src_q_index];
        }
        const Vec6 Sqd = j.S * qde;
        k.v[c] = k.X[c] * k.v[p] + Sqd;
        k.a[c] = k.X[c] * k.a[p] + j.S * qdde + crossm(k.v[c]) * Sqd;
    }
    return k;
}

} // namespace

DynModel build_dyn_model(const urm::RobotModel& model, const kin::CompiledModel& cm) {
    DynModel dm;
    dm.n_dof = cm.n_dof;
    dm.n_links = cm.n_links;
    dm.base = cm.base;
    dm.I.assign(cm.n_links, Mat6::Zero());
    for (int k = 0; k < cm.n_links && k < static_cast<int>(model.links.size()); ++k) {
        const urm::Inertia& in = model.links[k].inertia;
        const double m = in.mass;
        const Vector3d c(in.com[0], in.com[1], in.com[2]);
        Matrix3d Ic;
        Ic << in.inertia_tensor[0], in.inertia_tensor[1], in.inertia_tensor[2],
              in.inertia_tensor[1], in.inertia_tensor[3], in.inertia_tensor[4],
              in.inertia_tensor[2], in.inertia_tensor[4], in.inertia_tensor[5];
        const Matrix3d C = skew(c);
        Mat6 I = Mat6::Zero();
        I.topLeftCorner<3, 3>() = Ic + m * C * C.transpose();
        I.topRightCorner<3, 3>() = m * C;
        I.bottomLeftCorner<3, 3>() = m * C.transpose();
        I.bottomRightCorner<3, 3>() = m * Matrix3d::Identity();
        dm.I[k] = I;
    }
    for (const kin::CompiledJoint& cj : cm.joints) {
        DynJoint j;
        j.parent_link = cj.parent_link;
        j.child_link = cj.child_link;
        j.type = cj.type;
        j.src_q_index = cj.src_q_index;
        j.mult = cj.mult;
        j.offset = cj.offset;
        j.pre = cj.pre;
        j.post = cj.post;
        j.axis = cj.axis;
        const Matrix3d Rp = cj.post.linear();
        const Vector3d pp = cj.post.translation();
        if (cj.type == JointType::Revolute) {
            j.S.head<3>() = Rp.transpose() * cj.axis;
            j.S.tail<3>() = Rp.transpose() * cj.axis.cross(pp);
        } else if (cj.type == JointType::Prismatic) {
            j.S.tail<3>() = Rp.transpose() * cj.axis;
        }
        dm.joints.push_back(j);
    }
    return dm;
}

std::vector<double> inverse_dynamics(const DynModel& dm, const std::vector<double>& q,
                                     const std::vector<double>& qd, const std::vector<double>& qdd,
                                     const Vector3d& gravity_world, const Friction* friction) {
    check_size(dm, q, "q");
    check_size(dm, qd, "qd");
    check_size(dm, qdd, "qdd");
    const Kin k = forward_pass(dm, q, qd, qdd, gravity_world);
    std::vector<Vec6> f(dm.n_links, Vec6::Zero());
    for (const DynJoint& j : dm.joints) {
        const int c = j.child_link;
        f[c] = dm.I[c] * k.a[c] + crossf(k.v[c]) * (dm.I[c] * k.v[c]);
    }
    std::vector<double> tau(dm.n_dof, 0.0);
    for (auto it = dm.joints.rbegin(); it != dm.joints.rend(); ++it) {
        const DynJoint& j = *it;
        const int c = j.child_link;
        if (j.type != JointType::Fixed) tau[j.src_q_index] += j.mult * j.S.dot(f[c]);
        f[j.parent_link] += k.X[c].transpose() * f[c];
    }
    if (friction) {
        const std::vector<double> tf = friction_torque(dm, *friction, qd);
        for (int i = 0; i < dm.n_dof; ++i) tau[i] += tf[i];
    }
    return tau;
}

std::vector<double> friction_torque(const DynModel& dm, const Friction& f, const std::vector<double>& qd) {
    check_size(dm, qd, "qd");
    if (!f.viscous.empty() && static_cast<int>(f.viscous.size()) != dm.n_dof)
        throw std::invalid_argument("dynamics: friction.viscous needs n_dof entries");
    if (!f.coulomb.empty() && static_cast<int>(f.coulomb.size()) != dm.n_dof)
        throw std::invalid_argument("dynamics: friction.coulomb needs n_dof entries");
    if (!(f.eps > 0.0)) throw std::invalid_argument("dynamics: friction.eps must be > 0");
    std::vector<double> t(dm.n_dof, 0.0);
    for (int i = 0; i < dm.n_dof; ++i) {
        if (!f.viscous.empty()) t[i] += f.viscous[i] * qd[i];
        if (!f.coulomb.empty()) t[i] += f.coulomb[i] * std::tanh(qd[i] / f.eps);
    }
    return t;
}

Eigen::MatrixXd mass_matrix(const DynModel& dm, const std::vector<double>& q) {
    check_size(dm, q, "q");
    const int nj = static_cast<int>(dm.joints.size());
    std::vector<Mat6> X(dm.n_links, Mat6::Identity());
    std::vector<int> pj(dm.n_links, -1);   // compiled joint index of each link
    for (int i = 0; i < nj; ++i) {
        const DynJoint& j = dm.joints[i];
        X[j.child_link] = motion_xform_inv(joint_transform(j, q));
        pj[j.child_link] = i;
    }
    std::vector<Mat6> Ic = dm.I;
    for (int i = nj - 1; i >= 0; --i) {
        const DynJoint& j = dm.joints[i];
        Ic[j.parent_link] += X[j.child_link].transpose() * Ic[j.child_link] * X[j.child_link];
    }
    Eigen::MatrixXd M = Eigen::MatrixXd::Zero(dm.n_dof, dm.n_dof);
    for (int i = 0; i < nj; ++i) {
        const DynJoint& ji = dm.joints[i];
        if (ji.type == JointType::Fixed) continue;
        const int si = ji.src_q_index;
        Vec6 F = Ic[ji.child_link] * ji.S;
        M(si, si) += ji.mult * ji.mult * ji.S.dot(F);
        int j = i;
        while (dm.joints[j].parent_link != 0) {
            F = X[dm.joints[j].child_link].transpose() * F;
            j = pj[dm.joints[j].parent_link];
            const DynJoint& jj = dm.joints[j];
            if (jj.type != JointType::Fixed) {
                const double v = ji.mult * jj.mult * jj.S.dot(F);
                M(si, jj.src_q_index) += v;
                M(jj.src_q_index, si) += v;
            }
        }
    }
    return M;
}

std::vector<double> forward_dynamics(const DynModel& dm, const std::vector<double>& q,
                                     const std::vector<double>& qd, const std::vector<double>& tau,
                                     const Vector3d& gravity_world, const Friction* friction,
                                     const std::vector<double>* armature) {
    check_size(dm, q, "q");
    check_size(dm, qd, "qd");
    check_size(dm, tau, "tau");
    if (armature) check_size(dm, *armature, "armature");
    Eigen::MatrixXd M = mass_matrix(dm, q);
    if (armature)
        for (int i = 0; i < dm.n_dof; ++i) M(i, i) += (*armature)[i];
    const std::vector<double> b = bias_torque(dm, q, qd, gravity_world);
    std::vector<double> tf(dm.n_dof, 0.0);
    if (friction) tf = friction_torque(dm, *friction, qd);
    Eigen::VectorXd rhs(dm.n_dof);
    for (int i = 0; i < dm.n_dof; ++i) rhs[i] = tau[i] - tf[i] - b[i];
    Eigen::LLT<Eigen::MatrixXd> llt(M);
    if (llt.info() != Eigen::Success)
        throw std::runtime_error("mass matrix not positive definite: massless link or DOF with no inertia");
    const Eigen::VectorXd x = llt.solve(rhs);
    return std::vector<double>(x.data(), x.data() + x.size());
}

std::vector<double> gravity_torque(const DynModel& dm, const std::vector<double>& q, const Vector3d& g) {
    const std::vector<double> z(dm.n_dof, 0.0);
    return inverse_dynamics(dm, q, z, z, g);
}

std::vector<double> bias_torque(const DynModel& dm, const std::vector<double>& q,
                                const std::vector<double>& qd, const Vector3d& g) {
    const std::vector<double> z(dm.n_dof, 0.0);
    return inverse_dynamics(dm, q, qd, z, g);
}

std::vector<double> coriolis_torque(const DynModel& dm, const std::vector<double>& q,
                                    const std::vector<double>& qd, const Vector3d& g) {
    std::vector<double> b = bias_torque(dm, q, qd, g);
    const std::vector<double> gr = gravity_torque(dm, q, g);
    for (size_t i = 0; i < b.size(); ++i) b[i] -= gr[i];
    return b;
}

std::vector<Eigen::Isometry3d> dyn_frames(const DynModel& dm, const std::vector<double>& q) {
    check_size(dm, q, "q");
    std::vector<Eigen::Isometry3d> fr(dm.n_links, Eigen::Isometry3d::Identity());
    fr[0] = dm.base;
    for (const DynJoint& j : dm.joints) fr[j.child_link] = fr[j.parent_link] * joint_transform(j, q);
    return fr;
}

double kinetic_energy_spatial(const DynModel& dm, const std::vector<double>& q,
                              const std::vector<double>& qd) {
    check_size(dm, q, "q");
    check_size(dm, qd, "qd");
    const std::vector<double> z(dm.n_dof, 0.0);
    const Kin k = forward_pass(dm, q, qd, z, Vector3d::Zero());
    double ke = 0.0;
    for (const DynJoint& j : dm.joints)
        ke += 0.5 * k.v[j.child_link].dot(dm.I[j.child_link] * k.v[j.child_link]);
    return ke;
}

double kinetic_energy(const DynModel& dm, const std::vector<double>& q, const std::vector<double>& qd) {
    check_size(dm, qd, "qd");
    const Eigen::MatrixXd M = mass_matrix(dm, q);
    Eigen::VectorXd v(dm.n_dof);
    for (int i = 0; i < dm.n_dof; ++i) v[i] = qd[i];
    return 0.5 * v.dot(M * v);
}

double potential_energy(const DynModel& dm, const std::vector<double>& q, const Vector3d& g) {
    const std::vector<Eigen::Isometry3d> fr = dyn_frames(dm, q);
    double V = 0.0;
    for (int k = 0; k < dm.n_links; ++k) {
        // dm.I[k] layout: bottom-right = m * 1, top-right = m * skew(c_k) (COM in link frame).
        const double m = dm.I[k](3, 3);
        if (m == 0.0) continue;
        const Matrix3d S = dm.I[k].topRightCorner<3, 3>();
        const Vector3d mc(-S(1, 2), S(0, 2), -S(0, 1));   // m * c_k, link frame
        // m * p_world = R * (m c) + m * t
        const Vector3d mp = fr[k].linear() * mc + m * fr[k].translation();
        V -= g.dot(mp);
    }
    return V;
}

double total_energy(const DynModel& dm, const std::vector<double>& q, const std::vector<double>& qd,
                    const Vector3d& g) {
    return kinetic_energy(dm, q, qd) + potential_energy(dm, q, g);
}

std::vector<double> joint_power(const std::vector<double>& tau, const std::vector<double>& qd) {
    if (tau.size() != qd.size()) throw std::invalid_argument("joint_power: tau and qd sizes differ");
    std::vector<double> p(tau.size());
    for (size_t i = 0; i < tau.size(); ++i) p[i] = tau[i] * qd[i];
    return p;
}

} // namespace dyn
