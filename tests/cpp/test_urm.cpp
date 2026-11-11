/**
 * test_urm.cpp
 * R2.4: URM hardening tests.
 */

#include <iostream>
#include <vector>
#include <cmath>
#include <cassert>
#include <limits>
#include "urm/robot_model.hpp"
#include "urm/robot_model_io.hpp"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Test 1: Create a 3-DOF robot, check dof() == 3
bool test_create_3dof() {
    urm::RobotModel model;
    model.metadata.name = "3DOF_Arm";

    model.links.push_back(urm::Link{"base_link"});
    model.joints.push_back(urm::Joint{"joint1", urm::JointType::Revolute});
    model.links.push_back(urm::Link{"link1"});
    model.joints.push_back(urm::Joint{"joint2", urm::JointType::Revolute});
    model.links.push_back(urm::Link{"link2"});
    model.joints.push_back(urm::Joint{"joint3", urm::JointType::Revolute});
    model.links.push_back(urm::Link{"link3"});

    if (model.dof() != 3) {
        std::cerr << "Expected dof == 3, got " << model.dof() << "\n";
        return false;
    }
    std::cout << "PASS: test_create_3dof\n";
    return true;
}

// Test 2: set_q / get_q round-trip
bool test_set_get_q() {
    urm::RobotModel model;
    model.joints.push_back(urm::Joint{"j1", urm::JointType::Revolute});
    model.joints.push_back(urm::Joint{"j2", urm::JointType::Prismatic});
    model.joints.push_back(urm::Joint{"j3", urm::JointType::Revolute});

    std::vector<double> q_target = {0.5, 0.25, -1.2};
    model.set_q(q_target);

    std::vector<double> q_read = model.get_q();
    if (q_read.size() != 3) {
        std::cerr << "q_read size mismatch\n";
        return false;
    }
    for (size_t i = 0; i < 3; ++i) {
        if (std::abs(q_read[i] - q_target[i]) > 1e-9) {
            std::cerr << "Mismatch at index " << i << "\n";
            return false;
        }
    }
    std::cout << "PASS: test_set_get_q\n";
    return true;
}

// Test 3: set_q throws on wrong vector length
bool test_set_q_wrong_length_throws() {
    urm::RobotModel model;
    model.joints.push_back(urm::Joint{"j1", urm::JointType::Revolute});
    model.joints.push_back(urm::Joint{"j2", urm::JointType::Revolute});

    bool threw = false;
    try {
        model.set_q({1.0}); // only 1 element, expected 2
    } catch (const std::invalid_argument&) {
        threw = true;
    }
    if (!threw) {
        std::cerr << "Expected set_q({1.0}) to throw invalid_argument for 2 DOF model\n";
        return false;
    }

    threw = false;
    try {
        model.set_q({1.0, 2.0, 3.0}); // 3 elements, expected 2
    } catch (const std::invalid_argument&) {
        threw = true;
    }
    if (!threw) {
        std::cerr << "Expected set_q({1.0, 2.0, 3.0}) to throw invalid_argument for 2 DOF model\n";
        return false;
    }

    std::cout << "PASS: test_set_q_wrong_length_throws\n";
    return true;
}

// Test 4: JSON serialization round-trip
bool test_json_round_trip() {
    urm::RobotModel model;
    model.schema_version = 1;
    model.convention = urm::DHConvention::Modified;
    model.metadata.name = "Puma560";
    model.metadata.description = "6-DOF Industrial Manipulator";
    model.metadata.author = "Unimation";
    model.metadata.version = "2.0.0";
    model.base.xyz = {0.1, 0.2, 0.3};
    model.base.rpy = {0.01, 0.02, 0.03};
    model.tool.xyz = {0.0, 0.0, 0.15};
    model.tool.rpy = {0.0, 1.57, 0.0};

    for (int i = 0; i < 6; ++i) {
        model.links.push_back(urm::Link{"link_" + std::to_string(i)});
        urm::Joint j;
        j.name = "joint_" + std::to_string(i + 1);
        j.type = urm::JointType::Revolute;
        j.dh.a = 0.1 * (i + 1);
        j.dh.d = 0.2 * (i + 1);
        j.limits.lower = -3.14;
        j.limits.upper = 3.14;
        j.q = 0.05 * (i + 1);
        model.joints.push_back(j);
    }
    model.links.push_back(urm::Link{"link_6"}); // links = joints + 1

    std::string json_str = urm::to_json(model);
    if (json_str.empty()) {
        std::cerr << "JSON string is empty\n";
        return false;
    }

    urm::RobotModel loaded = urm::from_json(json_str);
    if (loaded.schema_version != 1) {
        std::cerr << "schema_version mismatch\n";
        return false;
    }
    if (loaded.convention != urm::DHConvention::Modified) {
        std::cerr << "convention mismatch\n";
        return false;
    }
    if (loaded.metadata.name != "Puma560" || loaded.dof() != 6) {
        std::cerr << "metadata or DOF mismatch\n";
        return false;
    }
    if (std::abs(loaded.base.xyz[0] - 0.1) > 1e-9 || std::abs(loaded.tool.xyz[2] - 0.15) > 1e-9) {
        std::cerr << "Base or tool pose mismatch\n";
        return false;
    }

    std::cout << "PASS: test_json_round_trip\n";
    return true;
}

// Test 5: schema_version behavior [R2.4-3]
bool test_schema_version() {
    // Missing schema_version in JSON -> defaults to 1
    std::string no_ver_json = R"({
        "metadata": {"name": "NoVerBot"},
        "joints": []
    })";
    urm::RobotModel m1 = urm::from_json(no_ver_json);
    if (m1.schema_version != 1) {
        std::cerr << "Expected missing schema_version to default to 1, got " << m1.schema_version << "\n";
        return false;
    }

    // schema_version > 1 throws runtime_error
    std::string v2_json = R"({
        "schema_version": 2,
        "metadata": {"name": "FutureBot"},
        "joints": []
    })";
    bool threw = false;
    try {
        urm::from_json(v2_json);
    } catch (const std::runtime_error&) {
        threw = true;
    }
    if (!threw) {
        std::cerr << "Expected schema_version 2 to throw runtime_error\n";
        return false;
    }

    // schema_version < 1 (e.g. 0) throws runtime_error [R2.4-3]
    std::string v0_json = R"({
        "schema_version": 0,
        "metadata": {"name": "PastBot"},
        "joints": []
    })";
    threw = false;
    try {
        urm::from_json(v0_json);
    } catch (const std::runtime_error&) {
        threw = true;
    }
    if (!threw) {
        std::cerr << "Expected schema_version 0 to throw runtime_error\n";
        return false;
    }

    std::cout << "PASS: test_schema_version\n";
    return true;
}

// Test 6: convention handling and legacy use_mdh
bool test_convention_and_legacy() {
    // Explicit convention: standard
    std::string std_json = R"({"convention": "standard", "joints": []})";
    urm::RobotModel m_std = urm::from_json(std_json);
    if (m_std.convention != urm::DHConvention::Standard) {
        std::cerr << "Expected convention Standard\n";
        return false;
    }

    // Explicit convention: modified
    std::string mod_json = R"({"convention": "modified", "joints": []})";
    urm::RobotModel m_mod = urm::from_json(mod_json);
    if (m_mod.convention != urm::DHConvention::Modified) {
        std::cerr << "Expected convention Modified\n";
        return false;
    }

    // Legacy JSON: all joints have use_mdh = true -> Modified
    std::string legacy_all_mdh = R"({
        "joints": [
            {"name": "j1", "type": "revolute", "use_mdh": true},
            {"name": "j2", "type": "revolute", "use_mdh": true}
        ]
    })";
    urm::RobotModel m_leg_mod = urm::from_json(legacy_all_mdh);
    if (m_leg_mod.convention != urm::DHConvention::Modified) {
        std::cerr << "Expected legacy all-mdh to deduce convention Modified\n";
        return false;
    }

    // Legacy JSON: no joints have use_mdh = true -> Standard
    std::string legacy_no_mdh = R"({
        "joints": [
            {"name": "j1", "type": "revolute"},
            {"name": "j2", "type": "revolute", "use_mdh": false}
        ]
    })";
    urm::RobotModel m_leg_std = urm::from_json(legacy_no_mdh);
    if (m_leg_std.convention != urm::DHConvention::Standard) {
        std::cerr << "Expected legacy no-mdh to deduce convention Standard\n";
        return false;
    }

    // Legacy JSON: mixed use_mdh -> throws runtime_error
    std::string legacy_mixed = R"({
        "joints": [
            {"name": "j1", "type": "revolute", "use_mdh": true},
            {"name": "j2", "type": "revolute", "use_mdh": false}
        ]
    })";
    bool threw = false;
    try {
        urm::from_json(legacy_mixed);
    } catch (const std::runtime_error&) {
        threw = true;
    }
    if (!threw) {
        std::cerr << "Expected mixed legacy use_mdh to throw runtime_error\n";
        return false;
    }

    std::cout << "PASS: test_convention_and_legacy\n";
    return true;
}

// Test 7: base and tool poses default to identity when missing
bool test_base_tool_poses() {
    std::string no_pose_json = R"({"metadata": {"name": "NoPoseBot"}, "joints": []})";
    urm::RobotModel m = urm::from_json(no_pose_json);
    for (int i = 0; i < 3; ++i) {
        if (m.base.xyz[i] != 0.0 || m.base.rpy[i] != 0.0 ||
            m.tool.xyz[i] != 0.0 || m.tool.rpy[i] != 0.0) {
            std::cerr << "Expected identity base and tool poses when missing\n";
            return false;
        }
    }
    std::cout << "PASS: test_base_tool_poses\n";
    return true;
}

// Test 8: default_limits function and JSON fallback [R2.4-2]
bool test_default_limits() {
    auto r_lim = urm::default_limits(urm::JointType::Revolute);
    if (std::abs(r_lim.lower - (-M_PI)) > 1e-9 || std::abs(r_lim.upper - M_PI) > 1e-9) {
        std::cerr << "Revolute default limits mismatch\n";
        return false;
    }

    auto p_lim = urm::default_limits(urm::JointType::Prismatic);
    if (std::abs(p_lim.lower - 0.0) > 1e-9 || std::abs(p_lim.upper - 0.5) > 1e-9) {
        std::cerr << "Prismatic default limits mismatch\n";
        return false;
    }

    auto f_lim = urm::default_limits(urm::JointType::Fixed);
    if (std::abs(f_lim.lower - 0.0) > 1e-9 || std::abs(f_lim.upper - 0.0) > 1e-9) {
        std::cerr << "Fixed default limits mismatch\n";
        return false;
    }

    // JSON without limits key uses default_limits
    std::string json_no_limits = R"({
        "joints": [
            {"name": "j_rev", "type": "revolute"},
            {"name": "j_prism", "type": "prismatic"},
            {"name": "j_fix", "type": "fixed"}
        ]
    })";
    urm::RobotModel m = urm::from_json(json_no_limits);
    if (std::abs(m.joints[0].limits.lower - (-M_PI)) > 1e-9 ||
        std::abs(m.joints[1].limits.upper - 0.5) > 1e-9 ||
        std::abs(m.joints[2].limits.upper - 0.0) > 1e-9) {
        std::cerr << "JSON fallback to default_limits failed\n";
        return false;
    }

    // [R2.4-2] Prismatic with partial limits (only vel specified) preserves default lower and upper
    std::string json_prism_vel = R"({
        "joints": [
            {"name": "j_prism_vel", "type": "prismatic", "limits": {"vel": 2.0}}
        ]
    })";
    urm::RobotModel m_pvel = urm::from_json(json_prism_vel);
    if (std::abs(m_pvel.joints[0].limits.lower - 0.0) > 1e-9 ||
        std::abs(m_pvel.joints[0].limits.upper - 0.5) > 1e-9 ||
        std::abs(m_pvel.joints[0].limits.vel - 2.0) > 1e-9 ||
        std::abs(m_pvel.joints[0].limits.effort - 100.0) > 1e-9) {
        std::cerr << "Prismatic partial limits override failed: got lower="
                  << m_pvel.joints[0].limits.lower << ", upper=" << m_pvel.joints[0].limits.upper
                  << ", vel=" << m_pvel.joints[0].limits.vel << "\n";
        return false;
    }

    std::cout << "PASS: test_default_limits\n";
    return true;
}

// Test 9: validate() rules
bool test_validate() {
    // 1. Valid model
    urm::RobotModel valid_m;
    valid_m.links.push_back(urm::Link{"base_link"});
    valid_m.joints.push_back(urm::Joint{"j1", urm::JointType::Revolute});
    valid_m.joints[0].limits = urm::default_limits(urm::JointType::Revolute);
    valid_m.joints[0].q = 0.0;
    valid_m.links.push_back(urm::Link{"link1"});
    auto errs = valid_m.validate();
    if (!errs.empty()) {
        std::cerr << "Expected valid model, got errors:\n";
        for (const auto& e : errs) std::cerr << " - " << e << "\n";
        return false;
    }

    // 2. Link count mismatch (1 joint, 1 link instead of 2 links)
    urm::RobotModel bad_links = valid_m;
    bad_links.links.pop_back();
    if (bad_links.validate().empty()) {
        std::cerr << "Expected link count error\n";
        return false;
    }

    // 3. lower > upper
    urm::RobotModel bad_limits = valid_m;
    bad_limits.joints[0].limits.lower = 2.0;
    bad_limits.joints[0].limits.upper = 1.0;
    if (bad_limits.validate().empty()) {
        std::cerr << "Expected lower > upper error\n";
        return false;
    }

    // 4. q outside limits
    urm::RobotModel bad_q = valid_m;
    bad_q.joints[0].limits = urm::default_limits(urm::JointType::Revolute);
    bad_q.joints[0].q = 4.0; // exceeds pi
    if (bad_q.validate().empty()) {
        std::cerr << "Expected q outside limits error\n";
        return false;
    }

    // 5. Duplicate joint names
    urm::RobotModel dup_j = valid_m;
    dup_j.links.push_back(urm::Link{"link2"});
    dup_j.joints.push_back(urm::Joint{"j1", urm::JointType::Revolute}); // duplicate "j1"
    dup_j.joints[1].limits = urm::default_limits(urm::JointType::Revolute);
    if (dup_j.validate().empty()) {
        std::cerr << "Expected duplicate joint name error\n";
        return false;
    }

    // 6. Non-finite number
    urm::RobotModel non_finite_m = valid_m;
    non_finite_m.joints[0].dh.a = std::numeric_limits<double>::quiet_NaN();
    if (non_finite_m.validate().empty()) {
        std::cerr << "Expected non-finite number error\n";
        return false;
    }

    std::cout << "PASS: test_validate\n";
    return true;
}

// Test 10: malformed JSON, root non-object, and type errors throw runtime_error [R2.4-1]
bool test_malformed_json_and_unknown_type() {
    auto check_throws_runtime_error = [](const std::string& input, const std::string& desc) -> bool {
        bool threw = false;
        try {
            urm::from_json(input);
        } catch (const std::runtime_error&) {
            threw = true;
        }
        if (!threw) {
            std::cerr << "Expected " << desc << " to throw std::runtime_error\n";
            return false;
        }
        return true;
    };

    // 1. Malformed JSON syntax
    if (!check_throws_runtime_error("{not valid json}", "malformed JSON syntax")) return false;

    // 2. Root is array, not object [R2.4-1]
    if (!check_throws_runtime_error("[]", "root array []")) return false;

    // 3. schema_version is string "2" (type error) [R2.4-1]
    if (!check_throws_runtime_error(R"({"schema_version": "2"})", "schema_version as string")) return false;

    // 4. joint q is string "abc" (type error) [R2.4-1]
    if (!check_throws_runtime_error(R"({"joints":[{"name":"j","type":"revolute","q":"abc"}]})", "joint q as string")) return false;

    // 5. inertia com is string "x" (type error) [R2.4-1]
    if (!check_throws_runtime_error(R"({"links":[{"inertia":{"com":"x"}}]})", "inertia com as string")) return false;

    // 6. Unknown joint type
    if (!check_throws_runtime_error(R"({"joints": [{"name": "j", "type": "spherical"}]})", "unknown joint type")) return false;

    std::cout << "PASS: test_malformed_json_and_unknown_type\n";
    return true;
}

// Test 11: Fixed joint not counted in dof()
bool test_fixed_joint_not_counted() {
    urm::RobotModel model;
    model.joints.push_back(urm::Joint{"base_mount", urm::JointType::Fixed});
    model.joints.push_back(urm::Joint{"shoulder", urm::JointType::Revolute});
    model.joints.push_back(urm::Joint{"elbow", urm::JointType::Revolute});
    model.joints.push_back(urm::Joint{"tool_fixed", urm::JointType::Fixed});

    if (model.dof() != 2) {
        std::cerr << "Expected dof == 2 with 2 fixed joints, got " << model.dof() << "\n";
        return false;
    }

    model.set_q({1.0, 2.0});
    auto q = model.get_q();
    if (q.size() != 2 || q[0] != 1.0 || q[1] != 2.0) {
        std::cerr << "get_q failed on model with fixed joints\n";
        return false;
    }

    std::cout << "PASS: test_fixed_joint_not_counted\n";
    return true;
}

int main() {
    int failures = 0;
    if (!test_create_3dof())                      { ++failures; }
    if (!test_set_get_q())                        { ++failures; }
    if (!test_set_q_wrong_length_throws())        { ++failures; }
    if (!test_json_round_trip())                  { ++failures; }
    if (!test_schema_version())                   { ++failures; }
    if (!test_convention_and_legacy())            { ++failures; }
    if (!test_base_tool_poses())                  { ++failures; }
    if (!test_default_limits())                   { ++failures; }
    if (!test_validate())                         { ++failures; }
    if (!test_malformed_json_and_unknown_type())  { ++failures; }
    if (!test_fixed_joint_not_counted())          { ++failures; }

    if (failures == 0) {
        std::cout << "\nAll URM hardening tests passed.\n";
        return 0;
    } else {
        std::cerr << "\n" << failures << " URM test(s) FAILED.\n";
        return 1;
    }
}
