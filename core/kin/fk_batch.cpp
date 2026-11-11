#include "fk_batch.hpp"

#include <stdexcept>
#include <unordered_map>

#include "compute/parallel.hpp"
#include "fk.hpp"

namespace kin {

std::vector<int> active_path(const CompiledModel& cm) {
    std::unordered_map<int, int> by_child;
    for (std::size_t k = 0; k < cm.joints.size(); ++k)
        by_child[cm.joints[k].child_link] = static_cast<int>(k);
    std::vector<int> path;
    int cur = cm.tcp_link;
    std::size_t guard = 0;
    while (cur != 0) {
        auto it = by_child.find(cur);
        if (it == by_child.end() || ++guard > cm.joints.size()) {
            throw std::invalid_argument("tcp_link is not reachable from link 0");
        }
        path.push_back(it->second);
        cur = cm.joints[it->second].parent_link;
    }
    return std::vector<int>(path.rbegin(), path.rend());
}

FkBatchResult fk_batch_cpu(const CompiledModel& cm, const double* q, std::size_t n,
                           std::size_t ld, int n_threads) {
    if (cm.n_dof > 0 && ld < n) {
        throw std::invalid_argument("fk_batch_cpu: ld must be >= n");
    }
    const std::vector<int> path = active_path(cm);
    FkBatchResult res;
    res.n = n;
    res.position.assign(3 * n, 0.0);
    res.rotation.assign(9 * n, 0.0);
    double* pos = res.position.data();
    double* rot = res.rotation.data();

    compute::parallel_for(n, n_threads, 2048, [&](std::size_t b, std::size_t e) {
        for (std::size_t i = b; i < e; ++i) {
            Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
            for (int idx : path) {
                const CompiledJoint& j = cm.joints[idx];
                Eigen::Isometry3d A = j.pre;
                if (j.type != urm::JointType::Fixed) {
                    const double qe =
                        j.mult * q[static_cast<std::size_t>(j.src_q_index) * ld + i] + j.offset;
                    if (j.type == urm::JointType::Revolute) {
                        A = A * Eigen::Isometry3d(Eigen::AngleAxisd(qe, j.axis));
                    } else {
                        A = A * Eigen::Isometry3d(Eigen::Translation3d(j.axis * qe));
                    }
                }
                A = A * j.post;
                T = T * A;
            }
            T = T * cm.tool;
            const Eigen::Matrix4d M = T.matrix();
            for (int c = 0; c < 3; ++c) pos[c * n + i] = M(c, 3);
            for (int r = 0; r < 3; ++r)
                for (int c = 0; c < 3; ++c) rot[(r * 3 + c) * n + i] = M(r, c);
        }
    });
    return res;
}

}  // namespace kin
