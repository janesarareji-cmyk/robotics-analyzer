#include "robot_model.hpp"
#include "robot_model_io.hpp"
#include <nlohmann/json.hpp>
#include <stdexcept>
#include <unordered_set>
#include <cmath>
#include <algorithm>
#include <Eigen/Dense>

using json = nlohmann::json;

namespace urm {

JointLimits default_limits(JointType type) {
    JointLimits lim;
    switch (type) {
        case JointType::Revolute:
            lim.lower  = -M_PI;
            lim.upper  =  M_PI;
            lim.vel    = 1.0;
            lim.effort = 100.0;
            break;
        case JointType::Prismatic:
            lim.lower  = 0.0;
            lim.upper  = 0.5;
            lim.vel    = 1.0;
            lim.effort = 100.0;
            break;
        case JointType::Fixed:
            lim.lower  = 0.0;
            lim.upper  = 0.0;
            lim.vel    = 1.0;
            lim.effort = 100.0;
            break;
    }
    return lim;
}

int RobotModel::dof() const {
    int count = 0;
    for (const auto& j : joints) {
        if (j.type != JointType::Fixed && !j.mimic.has_value()) {
            ++count;
        }
    }
    return count;
}

std::vector<int> RobotModel::q_index_map() const {
    std::vector<int> map(joints.size(), -1);
    int current_dof = 0;
    for (size_t i = 0; i < joints.size(); ++i) {
        if (joints[i].type != JointType::Fixed && !joints[i].mimic.has_value()) {
            map[i] = current_dof++;
        }
    }
    return map;
}

std::vector<double> RobotModel::get_q() const {
    std::vector<double> q_vec;
    q_vec.reserve(dof());
    for (const auto& j : joints) {
        if (j.type != JointType::Fixed && !j.mimic.has_value()) {
            q_vec.push_back(j.q);
        }
    }
    return q_vec;
}

void RobotModel::set_q(const std::vector<double>& q) {
    int expected = dof();
    if (static_cast<int>(q.size()) != expected) {
        throw std::invalid_argument("q size mismatch: expected " +
                                     std::to_string(expected) + ", got " +
                                     std::to_string(q.size()));
    }
    size_t idx = 0;
    for (auto& j : joints) {
        if (j.type != JointType::Fixed && !j.mimic.has_value()) {
            j.q = q[idx++];
        }
    }
}

std::vector<std::string> RobotModel::validate_physical() const {
    std::vector<std::string> msgs;
    // Which links are children of a non-fixed joint?
    std::vector<bool> moving(links.size(), false);
    for (const auto& j : joints) {
        if (j.type != JointType::Fixed && j.child_link >= 0 &&
            static_cast<size_t>(j.child_link) < links.size()) {
            moving[static_cast<size_t>(j.child_link)] = true;
        }
    }
    for (size_t i = 0; i < links.size(); ++i) {
        const auto& lk = links[i];
        const auto& in = lk.inertia;
        const std::string pre = "link '" + lk.name + "': ";
        bool finite = std::isfinite(in.mass);
        if (!finite) {
            msgs.push_back(pre + "mass is not finite");
        } else if (in.mass < 0.0) {
            msgs.push_back(pre + "mass is negative");
        }
        bool com_ok = true;
        for (double v : in.com) com_ok = com_ok && std::isfinite(v);
        if (!com_ok) msgs.push_back(pre + "com is not finite");
        bool t_ok = true;
        for (double v : in.inertia_tensor) t_ok = t_ok && std::isfinite(v);
        if (!t_ok) msgs.push_back(pre + "inertia tensor is not finite");

        if (finite && in.mass > 0.0 && t_ok) {
            const auto& t = in.inertia_tensor;
            Eigen::Matrix3d I;
            I << t[0], t[1], t[2],
                 t[1], t[3], t[4],
                 t[2], t[4], t[5];
            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(I);
            Eigen::Vector3d ev = es.eigenvalues();   // ascending
            if (ev[0] <= 1e-12) {
                msgs.push_back(pre + "inertia tensor is not positive definite");
            } else if (ev[0] + ev[1] < ev[2] - 1e-9) {
                msgs.push_back(pre + "principal moments violate the triangle inequality");
            }
        }
        if (i > 0 && moving[i] && finite && in.mass == 0.0) {
            msgs.push_back(pre + "mass not set");
        }
    }
    return msgs;
}

std::vector<std::string> RobotModel::validate() const {
    std::vector<std::string> errors;

    // Check link/joint count: links.size() == joints.size() + 1 (when joints non-empty)
    if (!joints.empty() && links.size() != joints.size() + 1) {
        errors.push_back("Link count must equal joint count + 1 (expected " +
                         std::to_string(joints.size() + 1) + ", got " +
                         std::to_string(links.size()) + ")");
    }

    // Check base and tool poses for finite numbers
    for (size_t i = 0; i < 3; ++i) {
        if (!std::isfinite(base.xyz[i])) {
            errors.push_back("Base pose xyz[" + std::to_string(i) + "] is not finite");
        }
        if (!std::isfinite(base.rpy[i])) {
            errors.push_back("Base pose rpy[" + std::to_string(i) + "] is not finite");
        }
        if (!std::isfinite(tool.xyz[i])) {
            errors.push_back("Tool pose xyz[" + std::to_string(i) + "] is not finite");
        }
        if (!std::isfinite(tool.rpy[i])) {
            errors.push_back("Tool pose rpy[" + std::to_string(i) + "] is not finite");
        }
    }

    // tcp_link: -1 or in range [0, links.size() - 1]
    if (tcp_link != -1) {
        if (links.empty() || tcp_link < 0 || tcp_link >= static_cast<int>(links.size())) {
            errors.push_back("tcp_link (" + std::to_string(tcp_link) +
                             ") is out of range [-1, " +
                             (links.empty() ? "-1" : std::to_string(links.size() - 1)) + "]");
        }
    }

    // Check joints
    std::unordered_set<std::string> seen_names;
    std::unordered_set<int> seen_children;

    for (size_t idx = 0; idx < joints.size(); ++idx) {
        const auto& j = joints[idx];
        std::string j_label = j.name.empty() ? ("joint[" + std::to_string(idx) + "]") : j.name;

        // Unique joint names
        if (j.name.empty()) {
            errors.push_back("Joint at index " + std::to_string(idx) + " has an empty name");
        } else if (!seen_names.insert(j.name).second) {
            errors.push_back("Duplicate joint name: '" + j.name + "'");
        }

        // Finite numbers
        if (!std::isfinite(j.limits.lower) || !std::isfinite(j.limits.upper) ||
            !std::isfinite(j.limits.vel) || !std::isfinite(j.limits.effort)) {
            errors.push_back("Joint '" + j_label + "' has non-finite limits");
        }
        if (!std::isfinite(j.dh.a) || !std::isfinite(j.dh.alpha) ||
            !std::isfinite(j.dh.d) || !std::isfinite(j.dh.theta)) {
            errors.push_back("Joint '" + j_label + "' has non-finite DH parameters");
        }
        if (!std::isfinite(j.mdh.a) || !std::isfinite(j.mdh.alpha) ||
            !std::isfinite(j.mdh.d) || !std::isfinite(j.mdh.theta)) {
            errors.push_back("Joint '" + j_label + "' has non-finite MDH parameters");
        }
        if (!std::isfinite(j.q)) {
            errors.push_back("Joint '" + j_label + "' has non-finite joint value q");
        }

        // lower <= upper
        if (std::isfinite(j.limits.lower) && std::isfinite(j.limits.upper) &&
            j.limits.lower > j.limits.upper) {
            errors.push_back("Joint '" + j_label + "': lower limit (" +
                             std::to_string(j.limits.lower) + ") exceeds upper limit (" +
                             std::to_string(j.limits.upper) + ")");
        }

        // q inside limits for non-fixed, non-mimic joints
        if (j.type != JointType::Fixed && !j.mimic.has_value() && std::isfinite(j.q) &&
            std::isfinite(j.limits.lower) && std::isfinite(j.limits.upper)) {
            if (j.q < j.limits.lower || j.q > j.limits.upper) {
                errors.push_back("Joint '" + j_label + "': value q (" +
                                 std::to_string(j.q) + ") outside limits [" +
                                 std::to_string(j.limits.lower) + ", " +
                                 std::to_string(j.limits.upper) + "]");
            }
        }

        // Parent and child links in range
        if (links.empty()) {
            errors.push_back("Joint '" + j_label + "': link list is empty");
        } else {
            if (j.parent_link < 0 || j.parent_link >= static_cast<int>(links.size())) {
                errors.push_back("Joint '" + j_label + "': parent_link (" +
                                 std::to_string(j.parent_link) + ") is out of range [0, " +
                                 std::to_string(links.size() - 1) + "]");
            }
            if (j.child_link < 0 || j.child_link >= static_cast<int>(links.size())) {
                errors.push_back("Joint '" + j_label + "': child_link (" +
                                 std::to_string(j.child_link) + ") is out of range [0, " +
                                 std::to_string(links.size() - 1) + "]");
            }
        }

        // parent_link < child_link
        if (j.parent_link >= j.child_link) {
            errors.push_back("Joint '" + j_label + "': parent_link (" +
                             std::to_string(j.parent_link) + ") must be less than child_link (" +
                             std::to_string(j.child_link) + ")");
        }

        // child_link != 0
        if (j.child_link == 0) {
            errors.push_back("Joint '" + j_label + "': child_link cannot be 0 (root link)");
        }

        // every child_link unique
        if (!seen_children.insert(j.child_link).second) {
            errors.push_back("Duplicate child_link " + std::to_string(j.child_link) +
                             " in joint '" + j_label + "'");
        }

        // origin_axis checks: axis finite and norm within 1e-6 of 1.0 (non-fixed joints)
        if (j.kinematics == JointKinematics::OriginAxis && j.type != JointType::Fixed) {
            if (!std::isfinite(j.axis[0]) || !std::isfinite(j.axis[1]) || !std::isfinite(j.axis[2])) {
                errors.push_back("Joint '" + j_label + "': axis contains non-finite values");
            } else {
                double norm = std::sqrt(j.axis[0]*j.axis[0] + j.axis[1]*j.axis[1] + j.axis[2]*j.axis[2]);
                if (std::abs(norm - 1.0) > 1e-6) {
                    errors.push_back("Joint '" + j_label + "': axis norm (" +
                                     std::to_string(norm) + ") is not unit length");
                }
            }
        }

        // Mimic joint validation
        if (j.mimic.has_value()) {
            const std::string& target_name = j.mimic->joint;
            bool target_found = false;
            for (const auto& other : joints) {
                if (other.name == target_name) {
                    target_found = true;
                    break;
                }
            }
            if (!target_found) {
                errors.push_back("Joint '" + j_label + "': mimic target joint '" +
                                 target_name + "' does not exist");
            } else if (std::any_of(joints.begin(), joints.end(), [&](const Joint& o) {
                           return o.name == target_name && o.type == JointType::Fixed; })) {
                errors.push_back("Joint '" + j_label + "': mimic target '" + target_name +
                                 "' is a fixed joint");
            } else if (target_name == j.name) {
                errors.push_back("Joint '" + j_label + "': mimic target cannot be itself");
            } else {
                // Check for cycles in mimic chain
                std::unordered_set<std::string> visited_mimic;
                visited_mimic.insert(j.name);
                std::string curr = target_name;
                bool cycle = false;
                while (!curr.empty()) {
                    if (visited_mimic.count(curr)) {
                        cycle = true;
                        break;
                    }
                    visited_mimic.insert(curr);
                    std::string next_target = "";
                    for (const auto& other : joints) {
                        if (other.name == curr && other.mimic.has_value()) {
                            next_target = other.mimic->joint;
                            break;
                        }
                    }
                    curr = next_target;
                }
                if (cycle) {
                    errors.push_back("Joint '" + j_label + "': cycle detected in mimic chain starting at '" +
                                     j.name + "'");
                }
            }
        }
    }

    // Tree connectivity: every link reachable from link 0
    if (!links.empty()) {
        std::vector<std::vector<int>> adj(links.size());
        for (const auto& j : joints) {
            if (j.parent_link >= 0 && j.parent_link < static_cast<int>(links.size()) &&
                j.child_link >= 0 && j.child_link < static_cast<int>(links.size())) {
                adj[j.parent_link].push_back(j.child_link);
            }
        }
        std::vector<bool> visited(links.size(), false);
        std::vector<int> queue = {0};
        visited[0] = true;
        size_t head = 0;
        while (head < queue.size()) {
            int u = queue[head++];
            for (int v : adj[u]) {
                if (!visited[v]) {
                    visited[v] = true;
                    queue.push_back(v);
                }
            }
        }
        if (queue.size() < links.size()) {
            for (size_t li = 0; li < links.size(); ++li) {
                if (!visited[li]) {
                    errors.push_back("Link " + std::to_string(li) + " ('" +
                                     links[li].name + "') is disconnected from root link 0");
                }
            }
        }
    }

    return errors;
}

static std::string joint_type_to_str(JointType type) {
    switch (type) {
        case JointType::Revolute:  return "revolute";
        case JointType::Prismatic: return "prismatic";
        case JointType::Fixed:     return "fixed";
    }
    return "revolute";
}

static std::string joint_kinematics_to_str(JointKinematics k) {
    switch (k) {
        case JointKinematics::DH:         return "dh";
        case JointKinematics::OriginAxis: return "origin_axis";
    }
    return "dh";
}

std::string to_json(const RobotModel& model, int indent) {
    json j;
    j["schema_version"] = 2;
    j["convention"] = (model.convention == DHConvention::Modified) ? "modified" : "standard";
    j["tcp_link"] = model.tcp_link;

    j["metadata"] = {
        {"name", model.metadata.name},
        {"description", model.metadata.description},
        {"author", model.metadata.author},
        {"version", model.metadata.version}
    };

    j["base"] = {
        {"xyz", model.base.xyz},
        {"rpy", model.base.rpy}
    };

    j["tool"] = {
        {"xyz", model.tool.xyz},
        {"rpy", model.tool.rpy}
    };

    j["links"] = json::array();
    for (const auto& link : model.links) {
        json jl;
        jl["name"] = link.name;
        jl["geometry_file"] = link.geometry_file;
        jl["inertia"] = {
            {"mass", link.inertia.mass},
            {"com", link.inertia.com},
            {"inertia_tensor", link.inertia.inertia_tensor}
        };
        j["links"].push_back(jl);
    }

    j["joints"] = json::array();
    for (const auto& joint : model.joints) {
        json jj;
        jj["name"] = joint.name;
        jj["type"] = joint_type_to_str(joint.type);
        jj["parent_link"] = joint.parent_link;
        jj["child_link"] = joint.child_link;
        jj["kinematics"] = joint_kinematics_to_str(joint.kinematics);
        jj["origin"] = {
            {"xyz", joint.origin.xyz},
            {"rpy", joint.origin.rpy}
        };
        jj["axis"] = joint.axis;
        if (joint.mimic.has_value()) {
            jj["mimic"] = {
                {"joint", joint.mimic->joint},
                {"multiplier", joint.mimic->multiplier},
                {"offset", joint.mimic->offset}
            };
        }
        jj["dh"] = {
            {"a", joint.dh.a},
            {"alpha", joint.dh.alpha},
            {"d", joint.dh.d},
            {"theta", joint.dh.theta}
        };
        jj["mdh"] = {
            {"a", joint.mdh.a},
            {"alpha", joint.mdh.alpha},
            {"d", joint.mdh.d},
            {"theta", joint.mdh.theta}
        };
        jj["limits"] = {
            {"lower", joint.limits.lower},
            {"upper", joint.limits.upper},
            {"vel", joint.limits.vel},
            {"effort", joint.limits.effort}
        };
        jj["q"] = joint.q;
        j["joints"].push_back(jj);
    }

    return j.dump(indent);
}

static void parse_pose(const json& j, const std::string& key, Pose& pose) {
    if (!j.contains(key)) {
        return;
    }
    if (!j[key].is_object()) {
        throw std::runtime_error("pose '" + key + "' must be an object");
    }
    const auto& jp = j[key];
    if (jp.contains("xyz")) {
        if (!jp["xyz"].is_array()) {
            throw std::runtime_error("pose '" + key + ".xyz' must be an array");
        }
        auto xyz_vec = jp["xyz"].get<std::vector<double>>();
        for (size_t i = 0; i < 3 && i < xyz_vec.size(); ++i) {
            pose.xyz[i] = xyz_vec[i];
        }
    }
    if (jp.contains("rpy")) {
        if (!jp["rpy"].is_array()) {
            throw std::runtime_error("pose '" + key + ".rpy' must be an array");
        }
        auto rpy_vec = jp["rpy"].get<std::vector<double>>();
        for (size_t i = 0; i < 3 && i < rpy_vec.size(); ++i) {
            pose.rpy[i] = rpy_vec[i];
        }
    }
}

RobotModel from_json(const std::string& json_str) {
    try {
        json j = json::parse(json_str);
        if (!j.is_object()) {
            throw std::runtime_error("invalid robot model JSON: root must be an object");
        }

        RobotModel model;

        // 1. schema_version check
        int version = 1;
        if (j.contains("schema_version")) {
            version = j["schema_version"].get<int>();
        }
        if (version < 1 || version > 2) {
            throw std::runtime_error("unsupported schema_version " + std::to_string(version));
        }
        model.schema_version = version;

        // 2. convention handling (with legacy use_mdh fallback)
        if (j.contains("convention")) {
            std::string conv_str = j.value("convention", "standard");
            if (conv_str == "standard") {
                model.convention = DHConvention::Standard;
            } else if (conv_str == "modified") {
                model.convention = DHConvention::Modified;
            } else {
                throw std::runtime_error("unknown convention: " + conv_str);
            }
        } else {
            // Legacy fallback: check each joint's "use_mdh"
            int mdh_count = 0;
            int joint_count = 0;
            if (j.contains("joints") && j["joints"].is_array()) {
                joint_count = static_cast<int>(j["joints"].size());
                for (const auto& jj : j["joints"]) {
                    if (jj.value("use_mdh", false)) {
                        ++mdh_count;
                    }
                }
            }
            if (joint_count > 0 && mdh_count == joint_count) {
                model.convention = DHConvention::Modified;
            } else if (mdh_count == 0) {
                model.convention = DHConvention::Standard;
            } else {
                throw std::runtime_error("mixed use_mdh in legacy JSON");
            }
        }

        // 3. tcp_link
        if (j.contains("tcp_link")) {
            model.tcp_link = j["tcp_link"].get<int>();
        } else {
            model.tcp_link = -1;
        }

        // 4. metadata
        if (j.contains("metadata")) {
            if (!j["metadata"].is_object()) {
                throw std::runtime_error("'metadata' must be an object");
            }
            const auto& m = j["metadata"];
            model.metadata.name = m.value("name", "Unnamed Robot");
            model.metadata.description = m.value("description", "");
            model.metadata.author = m.value("author", "");
            model.metadata.version = m.value("version", "1.0.0");
        }

        // 5. base and tool poses (default identity if missing)
        parse_pose(j, "base", model.base);
        parse_pose(j, "tool", model.tool);

        // 6. links
        if (j.contains("links")) {
            if (!j["links"].is_array()) {
                throw std::runtime_error("'links' must be an array");
            }
            for (const auto& jl : j["links"]) {
                if (!jl.is_object()) {
                    throw std::runtime_error("each link in 'links' must be an object");
                }
                Link link;
                link.name = jl.value("name", "");
                link.geometry_file = jl.value("geometry_file", "");
                if (jl.contains("inertia")) {
                    if (!jl["inertia"].is_object()) {
                        throw std::runtime_error("link 'inertia' must be an object");
                    }
                    const auto& in = jl["inertia"];
                    link.inertia.mass = in.value("mass", 0.0);
                    if (in.contains("com")) {
                        if (!in["com"].is_array()) {
                            throw std::runtime_error("link inertia 'com' must be an array");
                        }
                        auto com_vec = in["com"].get<std::vector<double>>();
                        for (size_t i = 0; i < 3 && i < com_vec.size(); ++i) {
                            link.inertia.com[i] = com_vec[i];
                        }
                    }
                    if (in.contains("inertia_tensor")) {
                        if (!in["inertia_tensor"].is_array()) {
                            throw std::runtime_error("link inertia 'inertia_tensor' must be an array");
                        }
                        auto it_vec = in["inertia_tensor"].get<std::vector<double>>();
                        for (size_t i = 0; i < 6 && i < it_vec.size(); ++i) {
                            link.inertia.inertia_tensor[i] = it_vec[i];
                        }
                    }
                }
                model.links.push_back(link);
            }
        }

        // 7. joints
        if (j.contains("joints")) {
            if (!j["joints"].is_array()) {
                throw std::runtime_error("'joints' must be an array");
            }
            for (const auto& jj : j["joints"]) {
                if (!jj.is_object()) {
                    throw std::runtime_error("each joint in 'joints' must be an object");
                }
                Joint joint;
                joint.name = jj.value("name", "");

                std::string type_str = jj.value("type", "revolute");
                if (type_str == "revolute") {
                    joint.type = JointType::Revolute;
                } else if (type_str == "prismatic") {
                    joint.type = JointType::Prismatic;
                } else if (type_str == "fixed") {
                    joint.type = JointType::Fixed;
                } else {
                    throw std::runtime_error("unknown joint type: " + type_str);
                }

                if (version == 2) {
                    joint.parent_link = jj.value("parent_link", 0);
                    joint.child_link = jj.value("child_link", 0);

                    if (jj.contains("kinematics")) {
                        std::string k_str = jj["kinematics"].get<std::string>();
                        if (k_str == "dh") {
                            joint.kinematics = JointKinematics::DH;
                        } else if (k_str == "origin_axis") {
                            joint.kinematics = JointKinematics::OriginAxis;
                        } else {
                            throw std::runtime_error("unknown kinematics: " + k_str);
                        }
                    }

                    parse_pose(jj, "origin", joint.origin);

                    if (jj.contains("axis")) {
                        if (!jj["axis"].is_array()) {
                            throw std::runtime_error("joint 'axis' must be an array");
                        }
                        auto ax = jj["axis"].get<std::vector<double>>();
                        for (size_t i = 0; i < 3 && i < ax.size(); ++i) {
                            joint.axis[i] = ax[i];
                        }
                    }

                    if (jj.contains("mimic") && !jj["mimic"].is_null()) {
                        if (!jj["mimic"].is_object()) {
                            throw std::runtime_error("joint 'mimic' must be an object");
                        }
                        const auto& jm = jj["mimic"];
                        Mimic m;
                        m.joint = jm.value("joint", "");
                        m.multiplier = jm.value("multiplier", 1.0);
                        m.offset = jm.value("offset", 0.0);
                        joint.mimic = m;
                    }
                }

                if (jj.contains("dh")) {
                    if (!jj["dh"].is_object()) {
                        throw std::runtime_error("joint 'dh' must be an object");
                    }
                    const auto& dh = jj["dh"];
                    joint.dh.a = dh.value("a", 0.0);
                    joint.dh.alpha = dh.value("alpha", 0.0);
                    joint.dh.d = dh.value("d", 0.0);
                    joint.dh.theta = dh.value("theta", 0.0);
                }

                if (jj.contains("mdh")) {
                    if (!jj["mdh"].is_object()) {
                        throw std::runtime_error("joint 'mdh' must be an object");
                    }
                    const auto& mdh = jj["mdh"];
                    joint.mdh.a = mdh.value("a", 0.0);
                    joint.mdh.alpha = mdh.value("alpha", 0.0);
                    joint.mdh.d = mdh.value("d", 0.0);
                    joint.mdh.theta = mdh.value("theta", 0.0);
                }

                // Start from default_limits and override only present keys
                joint.limits = default_limits(joint.type);
                if (jj.contains("limits")) {
                    if (!jj["limits"].is_object()) {
                        throw std::runtime_error("joint 'limits' must be an object");
                    }
                    const auto& jl = jj["limits"];
                    if (jl.contains("lower"))  joint.limits.lower  = jl["lower"].get<double>();
                    if (jl.contains("upper"))  joint.limits.upper  = jl["upper"].get<double>();
                    if (jl.contains("vel"))    joint.limits.vel    = jl["vel"].get<double>();
                    if (jl.contains("effort")) joint.limits.effort = jl["effort"].get<double>();
                }

                if (jj.contains("q")) {
                    joint.q = jj["q"].get<double>();
                }

                model.joints.push_back(joint);
            }
        }

        // Schema v1 in-memory migration
        if (version == 1) {
            for (size_t i = 0; i < model.joints.size(); ++i) {
                model.joints[i].parent_link = static_cast<int>(i);
                model.joints[i].child_link = static_cast<int>(i + 1);
                model.joints[i].kinematics = JointKinematics::DH;
                model.joints[i].origin = Pose{};
                model.joints[i].axis = {0.0, 0.0, 1.0};
                model.joints[i].mimic.reset();
            }
            model.tcp_link = model.links.empty() ? -1 : static_cast<int>(model.links.size()) - 1;
        }

        return model;
    } catch (const json::parse_error& exc) {
        throw std::runtime_error("invalid robot model JSON: " + std::string(exc.what()));
    } catch (const json::type_error& exc) {
        throw std::runtime_error("invalid robot model JSON type: " + std::string(exc.what()));
    }
}

} // namespace urm
