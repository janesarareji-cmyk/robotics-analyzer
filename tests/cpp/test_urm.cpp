/**
 * test_urm.cpp
 * R3.1: URM schema v2 tests: tree kinematics, origin/axis joints, mimic, q_index_map.
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
    model.schema_version = 2;
    model.convention = urm::DHConvention::Modified;
    model.metadata.name = "Puma560";
    model.metadata.description = "6-DOF Industrial Manipulator";
    model.metadata.author = "Unimation";
    model.metadata.version = "2.0.0";
    model.base.xyz = {0.1, 0.2, 0.3};
    model.base.rpy = {0.01, 0.02, 0.03};
    model.tool.xyz = {0.0, 0.0, 0.15};
    model.tool.rpy = {0.0, 1.57, 0.0};
    model.tcp_link = 6;

    for (int i = 0; i < 6; ++i) {
        model.links.push_back(urm::Link{"link_" + std::to_string(i)});
        urm::Joint j;
        j.name = "joint_" + std::to_string(i + 1);
        j.type = urm::JointType::Revolute;
        j.parent_link = i;
        j.child_link = i + 1;
        j.kinematics = urm::JointKinematics::DH;
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
    if (loaded.schema_version != 2) {
        std::cerr << "schema_version mismatch: got " << loaded.schema_version << "\n";
        return false;
    }
    if (loaded.convention != urm::DHConvention::Modified) {
        std::cerr << "convention mismatch\n";
        return false;
    }
    if (loaded.tcp_link != 6) {
        std::cerr << "tcp_link mismatch: got " << loaded.tcp_link << "\n";
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

// Test 5: schema_version behavior (v1 defaults to 1, v3 throws, v0 throws)
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

    // schema_version > 2 (e.g. 3) throws runtime_error
    std::string v3_json = R"({
        "schema_version": 3,
        "metadata": {"name": "FutureBot"},
        "joints": []
    })";
    bool threw = false;
    try {
        urm::from_json(v3_json);
    } catch (const std::runtime_error& e) {
        std::string msg = e.what();
        if (msg.find("unsupported schema_version 3") != std::string::npos) {
            threw = true;
        }
    }
    if (!threw) {
        std::cerr << "Expected schema_version 3 to throw runtime_error with 'unsupported schema_version 3'\n";
        return false;
    }

    // schema_version < 1 (e.g. 0) throws runtime_error
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

// Test 6: Convention and legacy use_mdh
bool test_convention_and_legacy() {
    std::string std_json = R"({"convention": "standard", "joints": []})";
    auto m_std = urm::from_json(std_json);
    if (m_std.convention != urm::DHConvention::Standard) return false;

    std::string mod_json = R"({"convention": "modified", "joints": []})";
    auto m_mod = urm::from_json(mod_json);
    if (m_mod.convention != urm::DHConvention::Modified) return false;

    std::string bad_conv = R"({"convention": "craig", "joints": []})";
    bool threw = false;
    try {
        urm::from_json(bad_conv);
    } catch (const std::runtime_error&) {
        threw = true;
    }
    if (!threw) return false;

    // Legacy fallback: all true -> modified
    std::string leg_mod = R"({"joints": [{"use_mdh": true}, {"use_mdh": true}]})";
    auto m_leg_mod = urm::from_json(leg_mod);
    if (m_leg_mod.convention != urm::DHConvention::Modified) return false;

    // Legacy fallback: all false/missing -> standard
    std::string leg_std = R"({"joints": [{"name": "j1"}, {"name": "j2"}]})";
    auto m_leg_std = urm::from_json(leg_std);
    if (m_leg_std.convention != urm::DHConvention::Standard) return false;

    // Legacy fallback: mixed -> throws
    std::string leg_mixed = R"({"joints": [{"use_mdh": true}, {"use_mdh": false}]})";
    threw = false;
    try {
        urm::from_json(leg_mixed);
    } catch (const std::runtime_error&) {
        threw = true;
    }
    if (!threw) return false;

    std::cout << "PASS: test_convention_and_legacy\n";
    return true;
}

// Test 7: Base and tool poses
bool test_base_tool_poses() {
    urm::RobotModel m;
    m.base.xyz = {1.0, 2.0, 3.0};
    m.base.rpy = {0.1, 0.2, 0.3};
    m.tool.xyz = {-0.1, -0.2, -0.3};
    m.tool.rpy = {-0.1, -0.2, -0.3};

    std::string s = urm::to_json(m);
    auto m2 = urm::from_json(s);
    if (std::abs(m2.base.xyz[0] - 1.0) > 1e-9 || std::abs(m2.base.xyz[1] - 2.0) > 1e-9 || std::abs(m2.base.xyz[2] - 3.0) > 1e-9) return false;
    if (std::abs(m2.base.rpy[0] - 0.1) > 1e-9 || std::abs(m2.base.rpy[1] - 0.2) > 1e-9 || std::abs(m2.base.rpy[2] - 0.3) > 1e-9) return false;
    if (std::abs(m2.tool.xyz[0] - (-0.1)) > 1e-9 || std::abs(m2.tool.xyz[1] - (-0.2)) > 1e-9 || std::abs(m2.tool.xyz[2] - (-0.3)) > 1e-9) return false;
    if (std::abs(m2.tool.rpy[0] - (-0.1)) > 1e-9 || std::abs(m2.tool.rpy[1] - (-0.2)) > 1e-9 || std::abs(m2.tool.rpy[2] - (-0.3)) > 1e-9) return false;

    std::cout << "PASS: test_base_tool_poses\n";
    return true;
}

// Test 8: Default limits
bool test_default_limits() {
    auto r_lim = urm::default_limits(urm::JointType::Revolute);
    if (std::abs(r_lim.lower - (-M_PI)) > 1e-9 || std::abs(r_lim.upper - M_PI) > 1e-9) return false;

    auto p_lim = urm::default_limits(urm::JointType::Prismatic);
    if (std::abs(p_lim.lower - 0.0) > 1e-9 || std::abs(p_lim.upper - 0.5) > 1e-9) return false;

    auto f_lim = urm::default_limits(urm::JointType::Fixed);
    if (std::abs(f_lim.lower - 0.0) > 1e-9 || std::abs(f_lim.upper - 0.0) > 1e-9) return false;

    std::cout << "PASS: test_default_limits\n";
    return true;
}

// Test 9: Validate R2.4 rules
bool test_validate() {
    urm::RobotModel m;
    m.links.push_back(urm::Link{"l0"});
    m.links.push_back(urm::Link{"l1"});
    urm::Joint j;
    j.name = "j1";
    j.parent_link = 0;
    j.child_link = 1;
    j.limits.lower = -1.0;
    j.limits.upper = 1.0;
    j.q = 0.5;
    m.joints.push_back(j);

    if (!m.validate().empty()) {
        std::cerr << "Expected valid model to pass validate()\n";
        return false;
    }

    // Invalid: lower > upper
    m.joints[0].limits.lower = 2.0;
    m.joints[0].limits.upper = 1.0;
    if (m.validate().empty()) return false;

    // Invalid: q outside limits
    m.joints[0].limits.lower = -1.0;
    m.joints[0].limits.upper = 1.0;
    m.joints[0].q = 2.0;
    if (m.validate().empty()) return false;

    // Invalid: duplicate name
    m.joints[0].q = 0.0;
    urm::Joint j2 = m.joints[0];
    m.joints.push_back(j2);
    m.links.push_back(urm::Link{"l2"});
    if (m.validate().empty()) return false;

    std::cout << "PASS: test_validate\n";
    return true;
}

// Test 10: Malformed JSON and unknown types
static bool check_throws_runtime_error(const std::string& json_str, const std::string& desc) {
    try {
        urm::from_json(json_str);
        std::cerr << "Expected runtime_error for: " << desc << "\n";
        return false;
    } catch (const std::runtime_error&) {
        return true;
    }
}

bool test_malformed_json_and_unknown_type() {
    if (!check_throws_runtime_error("{ invalid json", "syntax error")) return false;
    if (!check_throws_runtime_error("[1, 2, 3]", "root array")) return false;
    if (!check_throws_runtime_error(R"({"metadata": "not an object"})", "metadata not object")) return false;
    if (!check_throws_runtime_error(R"({"joints":[{"name":"j","type":"revolute","q":"abc"}]})", "joint q as string")) return false;
    if (!check_throws_runtime_error(R"({"links":[{"inertia":{"com":"x"}}]})", "inertia com as string")) return false;
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

// Test 12: v1 -> v2 migration values
bool test_v1_to_v2_migration() {
    std::string v1_json = R"({
        "schema_version": 1,
        "convention": "standard",
        "links": [{"name": "l0"}, {"name": "l1"}, {"name": "l2"}],
        "joints": [
            {"name": "j1", "type": "revolute", "dh": {"a": 0.5}},
            {"name": "j2", "type": "revolute", "dh": {"a": 0.4}}
        ]
    })";

    urm::RobotModel model = urm::from_json(v1_json);
    if (model.schema_version != 1) {
        std::cerr << "Expected model schema_version 1 after loading v1 JSON\n";
        return false;
    }
    if (model.tcp_link != 2) {
        std::cerr << "Expected tcp_link == 2 for 3 links, got " << model.tcp_link << "\n";
        return false;
    }
    if (model.joints[0].parent_link != 0 || model.joints[0].child_link != 1) {
        std::cerr << "Joint 0 parent/child link mismatch\n";
        return false;
    }
    if (model.joints[1].parent_link != 1 || model.joints[1].child_link != 2) {
        std::cerr << "Joint 1 parent/child link mismatch\n";
        return false;
    }
    if (model.joints[0].kinematics != urm::JointKinematics::DH) {
        std::cerr << "Expected DH kinematics for migrated joint\n";
        return false;
    }
    if (model.joints[0].axis[0] != 0.0 || model.joints[0].axis[1] != 0.0 || model.joints[0].axis[2] != 1.0) {
        std::cerr << "Expected axis [0,0,1] for migrated joint\n";
        return false;
    }
    if (model.joints[0].mimic.has_value()) {
        std::cerr << "Expected no mimic for migrated joint\n";
        return false;
    }

    auto errors = model.validate();
    if (!errors.empty()) {
        std::cerr << "Migrated model failed validation: " << errors[0] << "\n";
        return false;
    }

    std::cout << "PASS: test_v1_to_v2_migration\n";
    return true;
}

// Test 13: v2 round-trip with tree + OriginAxis + Mimic
bool test_v2_round_trip() {
    urm::RobotModel model;
    model.schema_version = 2;
    model.tcp_link = 2;
    model.links.push_back(urm::Link{"l0"});
    model.links.push_back(urm::Link{"l1"});
    model.links.push_back(urm::Link{"l2"});

    urm::Joint j1;
    j1.name = "j1";
    j1.type = urm::JointType::Revolute;
    j1.parent_link = 0;
    j1.child_link = 1;
    j1.kinematics = urm::JointKinematics::OriginAxis;
    j1.origin.xyz = {0.1, 0.2, 0.3};
    j1.origin.rpy = {0.0, 0.5, 0.0};
    j1.axis = {0.0, 1.0, 0.0};
    model.joints.push_back(j1);

    urm::Joint j2;
    j2.name = "j2";
    j2.type = urm::JointType::Revolute;
    j2.parent_link = 1;
    j2.child_link = 2;
    j2.kinematics = urm::JointKinematics::OriginAxis;
    j2.origin.xyz = {0.4, 0.0, 0.0};
    j2.axis = {0.0, 1.0, 0.0};
    urm::Mimic mi;
    mi.joint = "j1";
    mi.multiplier = 1.5;
    mi.offset = -0.1;
    j2.mimic = mi;
    model.joints.push_back(j2);

    std::string s = urm::to_json(model);
    urm::RobotModel loaded = urm::from_json(s);

    if (loaded.schema_version != 2 || loaded.tcp_link != 2) return false;
    if (loaded.joints.size() != 2) return false;
    const auto& lj1 = loaded.joints[0];
    if (lj1.parent_link != 0 || lj1.child_link != 1) return false;
    if (lj1.kinematics != urm::JointKinematics::OriginAxis) return false;
    if (std::abs(lj1.origin.xyz[0] - 0.1) > 1e-9 || std::abs(lj1.axis[1] - 1.0) > 1e-9) return false;

    const auto& lj2 = loaded.joints[1];
    if (!lj2.mimic.has_value()) return false;
    if (lj2.mimic->joint != "j1" || std::abs(lj2.mimic->multiplier - 1.5) > 1e-9 || std::abs(lj2.mimic->offset - (-0.1)) > 1e-9) return false;

    std::cout << "PASS: test_v2_round_trip\n";
    return true;
}

// Test 14: Unknown kinematics string throws
bool test_unknown_kinematics_throws() {
    std::string json_str = R"({
        "schema_version": 2,
        "joints": [{"name": "j", "kinematics": "unsupported_mode"}]
    })";
    bool threw = false;
    try {
        urm::from_json(json_str);
    } catch (const std::runtime_error& e) {
        std::string msg = e.what();
        if (msg.find("unknown kinematics: unsupported_mode") != std::string::npos) {
            threw = true;
        }
    }
    if (!threw) {
        std::cerr << "Expected unknown kinematics to throw runtime_error\n";
        return false;
    }

    std::cout << "PASS: test_unknown_kinematics_throws\n";
    return true;
}

// Test 15: DOF and q_index_map with fixed and mimic joints
bool test_dof_and_q_index_map() {
    urm::RobotModel model;
    // Chain: [revolute, fixed, revolute, mimic, prismatic]
    urm::Joint j1{"j1", urm::JointType::Revolute};
    urm::Joint j2{"j2", urm::JointType::Fixed};
    urm::Joint j3{"j3", urm::JointType::Revolute};
    urm::Joint j4{"j4", urm::JointType::Revolute};
    urm::Mimic mi{"j1", 1.0, 0.0};
    j4.mimic = mi;
    urm::Joint j5{"j5", urm::JointType::Prismatic};

    model.joints = {j1, j2, j3, j4, j5};

    if (model.dof() != 3) {
        std::cerr << "Expected dof == 3, got " << model.dof() << "\n";
        return false;
    }

    std::vector<int> expected_map = {0, -1, 1, -1, 2};
    std::vector<int> actual_map = model.q_index_map();
    if (actual_map != expected_map) {
        std::cerr << "q_index_map mismatch!\n";
        return false;
    }

    model.set_q({0.1, 0.2, 0.3});
    auto q = model.get_q();
    if (q.size() != 3 || q[0] != 0.1 || q[1] != 0.2 || q[2] != 0.3) {
        std::cerr << "get_q / set_q mismatch with mimic joint\n";
        return false;
    }

    std::cout << "PASS: test_dof_and_q_index_map\n";
    return true;
}

// Test 16: validate() new rules (pass + fail for each)
bool test_validate_new_rules() {
    // Base helper: 2-link 1-joint model
    auto make_valid = []() {
        urm::RobotModel m;
        m.links.push_back(urm::Link{"l0"});
        m.links.push_back(urm::Link{"l1"});
        urm::Joint j;
        j.name = "j1";
        j.parent_link = 0;
        j.child_link = 1;
        j.kinematics = urm::JointKinematics::OriginAxis;
        j.axis = {0.0, 0.0, 1.0};
        m.joints.push_back(j);
        return m;
    };

    // 1. parent_link / child_link range
    {
        auto m = make_valid();
        m.joints[0].parent_link = 5;
        if (m.validate().empty()) { std::cerr << "Failed: parent_link out of range should fail\n"; return false; }
        m.joints[0].parent_link = 0;
        m.joints[0].child_link = 5;
        if (m.validate().empty()) { std::cerr << "Failed: child_link out of range should fail\n"; return false; }
    }

    // 2. parent_link < child_link
    {
        auto m = make_valid();
        m.joints[0].parent_link = 1;
        m.joints[0].child_link = 1;
        if (m.validate().empty()) { std::cerr << "Failed: parent_link == child_link should fail\n"; return false; }
        m.joints[0].parent_link = 1;
        m.joints[0].child_link = 0;
        if (m.validate().empty()) { std::cerr << "Failed: parent_link > child_link should fail\n"; return false; }
    }

    // 3. child_link != 0
    {
        auto m = make_valid();
        m.joints[0].child_link = 0;
        m.joints[0].parent_link = -1; // so parent < child
        if (m.validate().empty()) { std::cerr << "Failed: child_link == 0 should fail\n"; return false; }
    }

    // 4. every child_link unique
    {
        auto m = make_valid();
        m.links.push_back(urm::Link{"l2"});
        urm::Joint j2 = m.joints[0];
        j2.name = "j2";
        j2.child_link = 1; // Duplicate child_link 1
        m.joints.push_back(j2);
        if (m.validate().empty()) { std::cerr << "Failed: duplicate child_link should fail\n"; return false; }
    }

    // 5. links.size() == joints.size() + 1
    {
        auto m = make_valid();
        m.links.push_back(urm::Link{"extra_link"});
        if (m.validate().empty()) { std::cerr << "Failed: link count mismatch should fail\n"; return false; }
    }

    // 6. axis finite and unit norm for OriginAxis non-fixed
    {
        auto m = make_valid();
        m.joints[0].axis = {0.0, 2.0, 0.0}; // norm = 2 != 1
        if (m.validate().empty()) { std::cerr << "Failed: non-unit axis norm should fail\n"; return false; }
        m.joints[0].axis = {std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0};
        if (m.validate().empty()) { std::cerr << "Failed: non-finite axis should fail\n"; return false; }
    }

    // 7. Mimic validation
    {
        // Target does not exist
        auto m = make_valid();
        m.joints[0].mimic = urm::Mimic{"nonexistent", 1.0, 0.0};
        if (m.validate().empty()) { std::cerr << "Failed: nonexistent mimic target should fail\n"; return false; }

        // Target is itself
        m.joints[0].mimic = urm::Mimic{"j1", 1.0, 0.0};
        if (m.validate().empty()) { std::cerr << "Failed: self-mimic should fail\n"; return false; }

        // Mimic cycle (j1 mimics j2, j2 mimics j1)
        m.links.push_back(urm::Link{"l2"});
        urm::Joint j2;
        j2.name = "j2";
        j2.parent_link = 1;
        j2.child_link = 2;
        j2.axis = {0.0, 0.0, 1.0};
        j2.kinematics = urm::JointKinematics::OriginAxis;
        j2.mimic = urm::Mimic{"j1", 1.0, 0.0};
        m.joints[0].mimic = urm::Mimic{"j2", 1.0, 0.0};
        m.joints.push_back(j2);
        if (m.validate().empty()) { std::cerr << "Failed: cyclic mimic chain should fail\n"; return false; }
    }

    // 8. tcp_link range
    {
        auto m = make_valid();
        m.tcp_link = -1; // Pass
        if (!m.validate().empty()) { std::cerr << "Failed: tcp_link -1 should pass\n"; return false; }
        m.tcp_link = 1;  // Pass
        if (!m.validate().empty()) { std::cerr << "Failed: tcp_link in range should pass\n"; return false; }
        m.tcp_link = 5;  // Fail
        if (m.validate().empty()) { std::cerr << "Failed: tcp_link out of range should fail\n"; return false; }
    }

    std::cout << "PASS: test_validate_new_rules\n";
    return true;
}

// Test 17: Branched tree (link0->j1->link1, link0->j2->link2, link1->j3->link3) validates clean
bool test_branched_tree_validates_clean() {
    urm::RobotModel model;
    model.links.push_back(urm::Link{"link0"});
    model.links.push_back(urm::Link{"link1"});
    model.links.push_back(urm::Link{"link2"});
    model.links.push_back(urm::Link{"link3"});

    // j1: link0 -> link1
    urm::Joint j1;
    j1.name = "j1";
    j1.parent_link = 0;
    j1.child_link = 1;
    j1.kinematics = urm::JointKinematics::OriginAxis;
    j1.axis = {0.0, 0.0, 1.0};

    // j2: link0 -> link2 (branch from root)
    urm::Joint j2;
    j2.name = "j2";
    j2.parent_link = 0;
    j2.child_link = 2;
    j2.kinematics = urm::JointKinematics::OriginAxis;
    j2.axis = {0.0, 1.0, 0.0};

    // j3: link1 -> link3 (child of link1)
    urm::Joint j3;
    j3.name = "j3";
    j3.parent_link = 1;
    j3.child_link = 3;
    j3.kinematics = urm::JointKinematics::OriginAxis;
    j3.axis = {1.0, 0.0, 0.0};

    model.joints = {j1, j2, j3};
    model.tcp_link = 3;

    auto errors = model.validate();
    if (!errors.empty()) {
        std::cerr << "Expected branched tree to validate cleanly, got: " << errors[0] << "\n";
        return false;
    }

    std::cout << "PASS: test_branched_tree_validates_clean\n";
    return true;
}

// Test 18: Disconnected link fails
bool test_disconnected_link_fails() {
    urm::RobotModel model;
    model.links.push_back(urm::Link{"link0"});
    model.links.push_back(urm::Link{"link1"});
    model.links.push_back(urm::Link{"link2"});
    model.links.push_back(urm::Link{"link3"});

    // j1: link0 -> link1
    urm::Joint j1;
    j1.name = "j1";
    j1.parent_link = 0;
    j1.child_link = 1;
    j1.axis = {0.0, 0.0, 1.0};
    j1.kinematics = urm::JointKinematics::OriginAxis;

    // j2: link1 -> link2
    urm::Joint j2;
    j2.name = "j2";
    j2.parent_link = 1;
    j2.child_link = 2;
    j2.axis = {0.0, 0.0, 1.0};
    j2.kinematics = urm::JointKinematics::OriginAxis;

    // j3: link1 -> link2 (duplicate child_link, so link3 is unreached)
    urm::Joint j3;
    j3.name = "j3";
    j3.parent_link = 1;
    j3.child_link = 2;
    j3.axis = {0.0, 0.0, 1.0};
    j3.kinematics = urm::JointKinematics::OriginAxis;

    model.joints = {j1, j2, j3};

    auto errors = model.validate();
    bool found_disconnected = false;
    for (const auto& err : errors) {
        if (err.find("disconnected from root link 0") != std::string::npos) {
            found_disconnected = true;
            break;
        }
    }
    if (!found_disconnected) {
        std::cerr << "Expected disconnected link error for unreached link 3\n";
        return false;
    }

    std::cout << "PASS: test_disconnected_link_fails\n";
    return true;
}

// Test 19 (R3.7): inertia defaults, round trip, missing -> mass 0, old v2 JSON opens
bool test_inertia_defaults_and_roundtrip() {
    urm::Link l;
    if (l.inertia.mass != 0.0) { std::cerr << "default mass must be 0\n"; return false; }
    for (double v : l.inertia.inertia_tensor)
        if (v != 0.0) { std::cerr << "default tensor must be zeros\n"; return false; }

    urm::RobotModel m;
    m.links.push_back(urm::Link{"l0"});
    urm::Link l1; l1.name = "l1";
    l1.inertia.mass = 2.5;
    l1.inertia.com = {0.1, -0.2, 0.3};
    l1.inertia.inertia_tensor = {1.0, 0.1, 0.2, 2.0, 0.3, 3.0};
    m.links.push_back(l1);
    urm::Joint j; j.name = "j"; j.parent_link = 0; j.child_link = 1;
    j.kinematics = urm::JointKinematics::OriginAxis; j.axis = {0,0,1};
    m.joints.push_back(j);
    auto m2 = urm::from_json(urm::to_json(m));
    const auto& a = m2.links[1].inertia;
    if (a.mass != 2.5 || a.com[0] != 0.1 || a.com[1] != -0.2 || a.com[2] != 0.3 ||
        a.inertia_tensor[1] != 0.1 || a.inertia_tensor[5] != 3.0) {
        std::cerr << "inertia round trip mismatch\n"; return false;
    }
    // missing inertia / missing mass -> 0
    auto m3 = urm::from_json(R"({"schema_version":2,"links":[{"name":"a"},{"name":"b","inertia":{"com":[1,2,3]}}]})");
    if (m3.links[0].inertia.mass != 0.0 || m3.links[1].inertia.mass != 0.0) {
        std::cerr << "missing mass must be 0\n"; return false;
    }
    if (m3.links[1].inertia.com[2] != 3.0) { std::cerr << "com not read\n"; return false; }
    // stored values in an old file are kept
    auto m4 = urm::from_json(R"({"schema_version":2,"links":[{"name":"a","inertia":{"mass":1.0,"inertia_tensor":[1,0,0,1,0,1]}}]})");
    if (m4.links[0].inertia.mass != 1.0 || m4.links[0].inertia.inertia_tensor[3] != 1.0) {
        std::cerr << "stored values must be kept\n"; return false;
    }
    std::cout << "PASS: test_inertia_defaults_and_roundtrip\n";
    return true;
}

// Test 20 (R3.7): validate_physical rules, validate() unaffected
bool test_validate_physical() {
    auto make = []() {
        urm::RobotModel m;
        m.links.push_back(urm::Link{"l0"});
        urm::Link l1; l1.name = "l1";
        l1.inertia.mass = 1.0;
        l1.inertia.inertia_tensor = {1,0,0,1,0,1};
        m.links.push_back(l1);
        urm::Joint j; j.name = "j1"; j.parent_link = 0; j.child_link = 1;
        j.kinematics = urm::JointKinematics::OriginAxis; j.axis = {0,0,1};
        m.joints.push_back(j);
        return m;
    };
    auto has = [](const std::vector<std::string>& v, const std::string& s) {
        for (const auto& x : v) if (x.find(s) != std::string::npos) return true;
        return false;
    };
    const double nan = std::numeric_limits<double>::quiet_NaN();
    {   auto m = make();
        if (!m.validate_physical().empty()) { std::cerr << "valid model must be clean\n"; return false; } }
    {   auto m = make(); m.links[1].inertia.mass = -1.0;
        auto v = m.validate_physical();
        if (!has(v, "negative") || !has(v, "'l1'")) { std::cerr << "negative mass\n"; return false; }
        if (!m.validate().empty()) { std::cerr << "validate() must ignore inertia\n"; return false; } }
    {   auto m = make(); m.links[1].inertia.mass = nan;
        if (!has(m.validate_physical(), "mass is not finite")) { std::cerr << "nan mass\n"; return false; }
        if (!m.validate().empty()) { std::cerr << "validate() must ignore nan mass\n"; return false; } }
    {   auto m = make(); m.links[1].inertia.com = {0, nan, 0};
        if (!has(m.validate_physical(), "com is not finite")) { std::cerr << "nan com\n"; return false; } }
    {   auto m = make(); m.links[1].inertia.inertia_tensor[3] = nan;
        if (!has(m.validate_physical(), "tensor is not finite")) { std::cerr << "nan tensor\n"; return false; } }
    {   auto m = make(); m.links[1].inertia.inertia_tensor = {1,0,0,-1,0,1};
        if (!has(m.validate_physical(), "positive definite")) { std::cerr << "not PD\n"; return false; }
        if (!m.validate().empty()) { std::cerr << "validate() must ignore bad tensor\n"; return false; } }
    {   auto m = make(); m.links[1].inertia.inertia_tensor = {1,0,0,1,0,0};
        if (!has(m.validate_physical(), "positive definite")) { std::cerr << "zero moment\n"; return false; } }
    {   // triangle inequality: 1 + 1 < 3
        auto m = make(); m.links[1].inertia.inertia_tensor = {1,0,0,1,0,3};
        if (!has(m.validate_physical(), "triangle")) { std::cerr << "triangle fail\n"; return false; }
        // boundary 1 + 1 == 2 passes
        m.links[1].inertia.inertia_tensor = {1,0,0,1,0,2};
        if (!m.validate_physical().empty()) { std::cerr << "triangle boundary must pass\n"; return false; } }
    {   // mass 0: tensor not checked, but "mass not set" for moving link
        auto m = make(); m.links[1].inertia = urm::Inertia{};
        auto v = m.validate_physical();
        if (v.size() != 1 || !has(v, "mass not set") || !has(v, "'l1'")) { std::cerr << "mass not set\n"; return false; }
        // fixed joint child: no message
        m.joints[0].type = urm::JointType::Fixed;
        if (!m.validate_physical().empty()) { std::cerr << "fixed child must not warn\n"; return false; } }
    std::cout << "PASS: test_validate_physical\n";
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
    if (!test_v1_to_v2_migration())               { ++failures; }
    if (!test_v2_round_trip())                    { ++failures; }
    if (!test_unknown_kinematics_throws())        { ++failures; }
    if (!test_dof_and_q_index_map())              { ++failures; }
    if (!test_validate_new_rules())               { ++failures; }
    if (!test_branched_tree_validates_clean())    { ++failures; }
    if (!test_disconnected_link_fails())          { ++failures; }
    if (!test_inertia_defaults_and_roundtrip())   { ++failures; }
    if (!test_validate_physical())                { ++failures; }

    if (failures == 0) {
        std::cout << "\nAll 20 URM tests passed.\n";
        return 0;
    } else {
        std::cerr << "\n" << failures << " URM test(s) FAILED.\n";
        return 1;
    }
}
