#include "dyn/dynamics.hpp"

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
                                     const Vector3d& gravity_world, const Friction* /*friction*/) {
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
    return tau;
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

} // namespace dyn
