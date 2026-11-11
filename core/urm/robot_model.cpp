#include "robot_model.hpp"
#include "robot_model_io.hpp"
#include <nlohmann/json.hpp>
#include <stdexcept>
#include <unordered_set>
#include <cmath>

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
        if (j.type != JointType::Fixed) {
            ++count;
        }
    }
    return count;
}

std::vector<double> RobotModel::get_q() const {
    std::vector<double> q_vec;
    q_vec.reserve(dof());
    for (const auto& j : joints) {
        if (j.type != JointType::Fixed) {
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
        if (j.type != JointType::Fixed) {
            j.q = q[idx++];
        }
    }
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

    // Check joint uniqueness, limits, and finite numbers
    std::unordered_set<std::string> seen_names;
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

        // q inside limits for non-fixed joints
        if (j.type != JointType::Fixed && std::isfinite(j.q) &&
            std::isfinite(j.limits.lower) && std::isfinite(j.limits.upper)) {
            if (j.q < j.limits.lower || j.q > j.limits.upper) {
                errors.push_back("Joint '" + j_label + "': value q (" +
                                 std::to_string(j.q) + ") outside limits [" +
                                 std::to_string(j.limits.lower) + ", " +
                                 std::to_string(j.limits.upper) + "]");
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

static JointType joint_type_from_str(const std::string& str) {
    if (str == "revolute")  return JointType::Revolute;
    if (str == "prismatic") return JointType::Prismatic;
    if (str == "fixed")     return JointType::Fixed;
    throw std::runtime_error("unknown joint type: " + str);
}

std::string to_json(const RobotModel& model, int indent) {
    json j;
    j["schema_version"] = model.schema_version;
    j["convention"] = (model.convention == DHConvention::Modified) ? "modified" : "standard";

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

        // 1. schema_version check [R2.4-3]
        int version = 1;
        if (j.contains("schema_version")) {
            version = j["schema_version"].get<int>();
        }
        if (version < 1 || version > 1) {
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

        // 3. metadata
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

        // 4. base and tool poses (default identity if missing)
        parse_pose(j, "base", model.base);
        parse_pose(j, "tool", model.tool);

        // 5. links
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
                    const auto& ji = jl["inertia"];
                    link.inertia.mass = ji.value("mass", 1.0);
                    if (ji.contains("com")) {
                        if (!ji["com"].is_array()) {
                            throw std::runtime_error("inertia 'com' must be an array");
                        }
                        auto com_vec = ji["com"].get<std::vector<double>>();
                        for (size_t i = 0; i < 3 && i < com_vec.size(); ++i) {
                            link.inertia.com[i] = com_vec[i];
                        }
                    }
                    if (ji.contains("inertia_tensor")) {
                        if (!ji["inertia_tensor"].is_array()) {
                            throw std::runtime_error("inertia 'inertia_tensor' must be an array");
                        }
                        auto tensor_vec = ji["inertia_tensor"].get<std::vector<double>>();
                        for (size_t i = 0; i < 6 && i < tensor_vec.size(); ++i) {
                            link.inertia.inertia_tensor[i] = tensor_vec[i];
                        }
                    }
                }
                model.links.push_back(link);
            }
        }

        // 6. joints
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
                std::string t_str = jj.value("type", "revolute");
                joint.type = joint_type_from_str(t_str);

                if (jj.contains("dh")) {
                    if (!jj["dh"].is_object()) {
                        throw std::runtime_error("joint 'dh' must be an object");
                    }
                    const auto& jdh = jj["dh"];
                    joint.dh.a = jdh.value("a", 0.0);
                    joint.dh.alpha = jdh.value("alpha", 0.0);
                    joint.dh.d = jdh.value("d", 0.0);
                    joint.dh.theta = jdh.value("theta", 0.0);
                }
                if (jj.contains("mdh")) {
                    if (!jj["mdh"].is_object()) {
                        throw std::runtime_error("joint 'mdh' must be an object");
                    }
                    const auto& jmdh = jj["mdh"];
                    joint.mdh.a = jmdh.value("a", 0.0);
                    joint.mdh.alpha = jmdh.value("alpha", 0.0);
                    joint.mdh.d = jmdh.value("d", 0.0);
                    joint.mdh.theta = jmdh.value("theta", 0.0);
                }

                // [R2.4-2] Start from default_limits and override only present keys
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

        return model;
    } catch (const std::runtime_error&) {
        throw; // Let our own std::runtime_error pass through unchanged
    } catch (const std::exception& e) {
        throw std::runtime_error(std::string("invalid robot model JSON: ") + e.what());
    }
}

} // namespace urm
