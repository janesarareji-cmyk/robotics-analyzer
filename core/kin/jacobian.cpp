#include "kin/jacobian.hpp"
#include "kin/fk.hpp"
#include <stdexcept>
#include <string>

namespace kin {

Eigen::MatrixXd jacobian(const CompiledModel& cm, const std::vector<double>& q,
                         int link, const Eigen::Vector3d& point_local) {
    if (link < 0 || link >= cm.n_links) {
        throw std::invalid_argument("jacobian: link index out of range");
    }
    const FKResult fk = forward_kinematics(cm, q);  // checks q size
    const Eigen::Vector3d p = fk.frames[link] * point_local;

    Eigen::MatrixXd J = Eigen::MatrixXd::Zero(6, cm.n_dof);

    // joint index (in cm.joints) by child link, to walk parents.
    std::vector<int> by_child(cm.n_links, -1);
    for (int k = 0; k < static_cast<int>(cm.joints.size()); ++k) {
        by_child[cm.joints[k].child_link] = k;
    }
    int cur = link;
    while (cur != 0) {
        const int k = by_child[cur];
        if (k < 0) break;  // unreachable from base: no contribution
        const CompiledJoint& j = cm.joints[k];
        cur = j.parent_link;
        if (j.type == urm::JointType::Fixed) continue;
        const Eigen::Isometry3d F = fk.frames[j.parent_link] * j.pre;
        const Eigen::Vector3d z = F.linear() * j.axis;
        const Eigen::Vector3d o = F.translation();
        Eigen::Matrix<double, 6, 1> col;
        if (j.type == urm::JointType::Revolute) {
            col.head<3>() = z.cross(p - o);
            col.tail<3>() = z;
        } else {
            col.head<3>() = z;
            col.tail<3>().setZero();
        }
        J.col(j.src_q_index) += j.mult * col;
    }
    return J;
}

Eigen::MatrixXd jacobian_tcp(const CompiledModel& cm, const std::vector<double>& q) {
    return jacobian(cm, q, cm.tcp_link, cm.tool.translation());
}

} // namespace kin
