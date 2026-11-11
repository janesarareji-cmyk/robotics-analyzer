/**
 * test_urm.cpp
 * R2.1: URM schema creation and JSON round-trip tests.
 */

#include <iostream>
#include <vector>
#include <cmath>
#include "urm/robot_model.hpp"
#include "urm/robot_model_io.hpp"

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

// Test 3: JSON serialization: serialize then deserialize, compare name and dof
bool test_json_round_trip() {
    urm::RobotModel model;
    model.metadata.name = "Puma560";
    model.metadata.description = "6-DOF Industrial Manipulator";
    model.metadata.author = "Unimation";
    model.metadata.version = "2.0.0";

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

    std::string json_str = urm::to_json(model);
    if (json_str.empty()) {
        std::cerr << "JSON string is empty\n";
        return false;
    }

    urm::RobotModel loaded = urm::from_json(json_str);
    if (loaded.metadata.name != "Puma560") {
        std::cerr << "Name mismatch: " << loaded.metadata.name << "\n";
        return false;
    }
    if (loaded.metadata.description != "6-DOF Industrial Manipulator") {
        std::cerr << "Description mismatch\n";
        return false;
    }
    if (loaded.dof() != 6) {
        std::cerr << "DOF mismatch: " << loaded.dof() << "\n";
        return false;
    }
    if (loaded.links.size() != 6 || loaded.joints.size() != 6) {
        std::cerr << "Link or joint count mismatch\n";
        return false;
    }
    for (size_t i = 0; i < 6; ++i) {
        if (std::abs(loaded.joints[i].dh.a - model.joints[i].dh.a) > 1e-9) {
            std::cerr << "DH parameter 'a' mismatch at joint " << i << "\n";
            return false;
        }
        if (std::abs(loaded.joints[i].q - model.joints[i].q) > 1e-9) {
            std::cerr << "Joint q mismatch at joint " << i << "\n";
            return false;
        }
    }

    std::cout << "PASS: test_json_round_trip\n";
    return true;
}

// Test 4: Fixed joint not counted in dof()
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
    if (!test_create_3dof())             { ++failures; }
    if (!test_set_get_q())               { ++failures; }
    if (!test_json_round_trip())         { ++failures; }
    if (!test_fixed_joint_not_counted()) { ++failures; }

    if (failures == 0) {
        std::cout << "\nAll URM tests passed.\n";
        return 0;
    } else {
        std::cerr << "\n" << failures << " URM test(s) FAILED.\n";
        return 1;
    }
}
