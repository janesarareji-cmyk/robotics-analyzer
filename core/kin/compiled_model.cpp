#include "compiled_model.hpp"
#include "fk.hpp"
#include <algorithm>
#include <stdexcept>
#include <string>
#include <unordered_map>

namespace kin {

using urm::JointKinematics;
using urm::JointType;

std::vector<std::pair<int, int>> effective_links(const urm::RobotModel& model) {
    const size_t n = model.joints.size();
    std::vector<std::pair<int, int>> out(n);
    bool legacy = n >= 1;
    for (const auto& j : model.joints) {
        if (j.parent_link != 0 || j.child_link != 0) { legacy = false; break; }
    }
    if (legacy) {
        if (model.links.size() != n + 1) {
            throw std::invalid_argument(
                "legacy serial model: links.size() must equal joints.size() + 1");
        }
        for (size_t i = 0; i < n; ++i) {
            out[i] = {static_cast<int>(i), static_cast<int>(i) + 1};
        }
    } else {
        for (size_t i = 0; i < n; ++i) {
            out[i] = {model.joints[i].parent_link, model.joints[i].child_link};
        }
    }
    return out;
}

static Eigen::Isometry3d rz(double t) {
    return Eigen::Isometry3d(Eigen::AngleAxisd(t, Eigen::Vector3d::UnitZ()));
}
static Eigen::Isometry3d rx(double t) {
    return Eigen::Isometry3d(Eigen::AngleAxisd(t, Eigen::Vector3d::UnitX()));
}
static Eigen::Isometry3d tx(double v) {
    return Eigen::Isometry3d(Eigen::Translation3d(v, 0.0, 0.0));
}
static Eigen::Isometry3d tz(double v) {
    return Eigen::Isometry3d(Eigen::Translation3d(0.0, 0.0, v));
}

static int resolve_tcp(const urm::RobotModel& model, int n_links) {
    int t = model.tcp_link == -1 ? n_links - 1 : model.tcp_link;
    if (t < 0 || t >= n_links) {
        throw std::invalid_argument("tcp_link out of range");
    }
    return t;
}

CompiledModel compile(const urm::RobotModel& model) {
    const int nj = static_cast<int>(model.joints.size());
    const auto eff = effective_links(model);

    CompiledModel cm;
    cm.n_links = (model.links.empty() && nj == 0) ? 1 : static_cast<int>(model.links.size());
    cm.base = pose_to_iso(model.base);
    cm.tool = pose_to_iso(model.tool);
    cm.tcp_link = resolve_tcp(model, cm.n_links);

    const std::vector<int> qmap = model.q_index_map();
    cm.n_dof = model.dof();

    std::unordered_map<std::string, int> by_name;
    for (int i = 0; i < nj; ++i) by_name.emplace(model.joints[i].name, i);

    std::vector<bool> child_used(cm.n_links, false);
    cm.joints.reserve(nj);

    for (int i = 0; i < nj; ++i) {
        const urm::Joint& j = model.joints[i];
        const int p = eff[i].first, c = eff[i].second;
        if (p < 0 || p >= cm.n_links || c < 0 || c >= cm.n_links) {
            throw std::invalid_argument("joint '" + j.name + "': link index out of range");
        }
        if (p >= c) {
            throw std::invalid_argument("joint '" + j.name + "': parent_link must be less than child_link");
        }
        if (child_used[c]) {
            throw std::invalid_argument("joint '" + j.name + "': duplicate child_link");
        }
        child_used[c] = true;

        CompiledJoint cj;
        cj.joint_index = i;
        cj.parent_link = p;
        cj.child_link = c;
        cj.type = j.type;
        cj.pre = Eigen::Isometry3d::Identity();
        cj.post = Eigen::Isometry3d::Identity();
        cj.axis = Eigen::Vector3d::UnitZ();
        cj.q_index = qmap[i];
        cj.src_q_index = -1;
        cj.mult = 1.0;
        cj.offset = 0.0;

        // Geometry
        if (j.kinematics == JointKinematics::OriginAxis) {
            cj.pre = pose_to_iso(j.origin);
            if (j.type != JointType::Fixed) {
                Eigen::Vector3d ax(j.axis[0], j.axis[1], j.axis[2]);
                double n = ax.norm();
                if (!(n > 0.0)) {
                    throw std::invalid_argument("joint '" + j.name + "': zero axis");
                }
                cj.axis = ax / n;
            }
        } else if (model.convention == urm::DHConvention::Standard) {
            const auto& d = j.dh;
            if (j.type == JointType::Fixed) {
                cj.pre = dh_transform(d.a, d.alpha, d.d, d.theta);
            } else if (j.type == JointType::Revolute) {
                cj.pre = rz(d.theta);
                cj.post = tz(d.d) * tx(d.a) * rx(d.alpha);
            } else {
                cj.pre = rz(d.theta) * tz(d.d);
                cj.post = tx(d.a) * rx(d.alpha);
            }
        } else {
            const auto& d = j.mdh;
            if (j.type == JointType::Fixed) {
                cj.pre = mdh_transform(d.a, d.alpha, d.d, d.theta);
            } else if (j.type == JointType::Revolute) {
                cj.pre = rx(d.alpha) * tx(d.a) * rz(d.theta);
                cj.post = tz(d.d);
            } else {
                cj.pre = rx(d.alpha) * tx(d.a) * rz(d.theta) * tz(d.d);
            }
        }

        // q source (mimic chain flattened to root master)
        if (j.type != JointType::Fixed) {
            if (!j.mimic.has_value()) {
                cj.src_q_index = qmap[i];
            } else {
                double A = 1.0, B = 0.0;
                int cur = i;
                int steps = 0;
                while (model.joints[cur].mimic.has_value()) {
                    if (++steps > nj) {
                        throw std::invalid_argument("joint '" + j.name + "': mimic cycle");
                    }
                    const urm::Mimic& m = *model.joints[cur].mimic;
                    auto it = by_name.find(m.joint);
                    if (it == by_name.end()) {
                        throw std::invalid_argument("joint '" + j.name + "': mimic target '" + m.joint + "' not found");
                    }
                    B = A * m.offset + B;
                    A = A * m.multiplier;
                    cur = it->second;
                }
                if (model.joints[cur].type == JointType::Fixed) {
                    throw std::invalid_argument("joint '" + j.name + "': mimic chain ends on fixed joint '" +
                                                model.joints[cur].name + "'");
                }
                cj.src_q_index = qmap[cur];
                cj.mult = A;
                cj.offset = B;
            }
        }
        cm.joints.push_back(cj);
    }

    std::sort(cm.joints.begin(), cm.joints.end(),
                     [](const CompiledJoint& a, const CompiledJoint& b) { return a.child_link < b.child_link; });
    return cm;
}

std::vector<int> active_chain(const urm::RobotModel& model) {
    const auto eff = effective_links(model);
    const int n_links = (model.links.empty() && model.joints.empty()) ? 1 : static_cast<int>(model.links.size());
    int cur = resolve_tcp(model, n_links);
    std::unordered_map<int, int> joint_of_child;
    for (size_t i = 0; i < eff.size(); ++i) joint_of_child[eff[i].second] = static_cast<int>(i);

    std::vector<int> chain;
    size_t guard = 0;
    while (cur != 0) {
        auto it = joint_of_child.find(cur);
        if (it == joint_of_child.end() || ++guard > eff.size()) {
            throw std::invalid_argument("tcp_link is not reachable from link 0");
        }
        chain.push_back(it->second);
        cur = eff[it->second].first;
    }
    std::reverse(chain.begin(), chain.end());
    return chain;
}

} // namespace kin
