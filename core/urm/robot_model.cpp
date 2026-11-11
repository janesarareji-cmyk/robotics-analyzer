#include "robot_model.hpp"
#include "robot_model_io.hpp"
#include <nlohmann/json.hpp>

using json = nlohmann::json;

namespace urm {

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
    size_t idx = 0;
    for (auto& j : joints) {
        if (j.type != JointType::Fixed) {
            if (idx < q.size()) {
                j.q = q[idx++];
            }
        }
    }
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
    if (str == "prismatic") return JointType::Prismatic;
    if (str == "fixed")     return JointType::Fixed;
    return JointType::Revolute;
}

std::string to_json(const RobotModel& model, int indent) {
    json j;
    j["metadata"] = {
        {"name", model.metadata.name},
        {"description", model.metadata.description},
        {"author", model.metadata.author},
        {"version", model.metadata.version}
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
        jj["use_mdh"] = joint.use_mdh;
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

RobotModel from_json(const std::string& json_str) {
    RobotModel model;
    auto j = json::parse(json_str);

    if (j.contains("metadata")) {
        const auto& m = j["metadata"];
        model.metadata.name = m.value("name", "Unnamed Robot");
        model.metadata.description = m.value("description", "");
        model.metadata.author = m.value("author", "");
        model.metadata.version = m.value("version", "1.0.0");
    }

    if (j.contains("links") && j["links"].is_array()) {
        for (const auto& jl : j["links"]) {
            Link link;
            link.name = jl.value("name", "");
            link.geometry_file = jl.value("geometry_file", "");
            if (jl.contains("inertia")) {
                const auto& ji = jl["inertia"];
                link.inertia.mass = ji.value("mass", 1.0);
                if (ji.contains("com") && ji["com"].is_array()) {
                    auto com_vec = ji["com"].get<std::vector<double>>();
                    for (size_t i = 0; i < 3 && i < com_vec.size(); ++i) {
                        link.inertia.com[i] = com_vec[i];
                    }
                }
                if (ji.contains("inertia_tensor") && ji["inertia_tensor"].is_array()) {
                    auto tensor_vec = ji["inertia_tensor"].get<std::vector<double>>();
                    for (size_t i = 0; i < 6 && i < tensor_vec.size(); ++i) {
                        link.inertia.inertia_tensor[i] = tensor_vec[i];
                    }
                }
            }
            model.links.push_back(link);
        }
    }

    if (j.contains("joints") && j["joints"].is_array()) {
        for (const auto& jj : j["joints"]) {
            Joint joint;
            joint.name = jj.value("name", "");
            joint.type = joint_type_from_str(jj.value("type", "revolute"));
            joint.use_mdh = jj.value("use_mdh", false);
            if (jj.contains("dh")) {
                const auto& jdh = jj["dh"];
                joint.dh.a = jdh.value("a", 0.0);
                joint.dh.alpha = jdh.value("alpha", 0.0);
                joint.dh.d = jdh.value("d", 0.0);
                joint.dh.theta = jdh.value("theta", 0.0);
            }
            if (jj.contains("mdh")) {
                const auto& jmdh = jj["mdh"];
                joint.mdh.a = jmdh.value("a", 0.0);
                joint.mdh.alpha = jmdh.value("alpha", 0.0);
                joint.mdh.d = jmdh.value("d", 0.0);
                joint.mdh.theta = jmdh.value("theta", 0.0);
            }
            if (jj.contains("limits")) {
                const auto& jl = jj["limits"];
                joint.limits.lower = jl.value("lower", -M_PI);
                joint.limits.upper = jl.value("upper", M_PI);
                joint.limits.vel = jl.value("vel", 1.0);
                joint.limits.effort = jl.value("effort", 100.0);
            }
            joint.q = jj.value("q", 0.0);
            model.joints.push_back(joint);
        }
    }

    return model;
}

} // namespace urm
