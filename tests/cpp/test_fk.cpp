/**
 * test_fk.cpp
 * R2.5: Forward Kinematics (FK) C++ unit tests.
 */

#include <iostream>
#include <vector>
#include <cmath>
#include <random>
#include <cassert>

#include "kin/fk.hpp"
#include "urm/robot_model.hpp"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static double wrap_to_pi(double angle) {
    while (angle > M_PI)  angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

// 1. 1R: a=1, q=π/2 → TCP xyz = (0, 1, 0)
bool test_1r() {
    urm::RobotModel model;
    model.links.push_back(urm::Link{"base_link"});
    urm::Joint j;
    j.name = "j1";
    j.type = urm::JointType::Revolute;
    j.dh.a = 1.0;
    j.q = M_PI / 2.0;
    model.joints.push_back(j);
    model.links.push_back(urm::Link{"link1"});

    kin::FKResult res = kin::forward_kinematics(model);
    Eigen::Vector3d pos = res.tcp.translation();

    if (std::abs(pos.x() - 0.0) > 1e-12 ||
        std::abs(pos.y() - 1.0) > 1e-12 ||
        std::abs(pos.z() - 0.0) > 1e-12) {
        std::cerr << "1R position mismatch: (" << pos.x() << ", " << pos.y() << ", " << pos.z() << ")\n";
        return false;
    }
    std::cout << "PASS: test_1r\n";
    return true;
}

// 2. 2R planar a1=a2=1: q=(0,0) → (2,0,0); q=(π/2,−π/2) → (1,1,0)
bool test_2r_planar() {
    urm::RobotModel model;
    model.links.push_back(urm::Link{"base_link"});
    for (int i = 1; i <= 2; ++i) {
        urm::Joint j;
        j.name = "j" + std::to_string(i);
        j.type = urm::JointType::Revolute;
        j.dh.a = 1.0;
        model.joints.push_back(j);
        model.links.push_back(urm::Link{"link" + std::to_string(i)});
    }

    // q = (0, 0)
    kin::FKResult res0 = kin::forward_kinematics(model, {0.0, 0.0});
    Eigen::Vector3d p0 = res0.tcp.translation();
    if (std::abs(p0.x() - 2.0) > 1e-12 || std::abs(p0.y() - 0.0) > 1e-12 || std::abs(p0.z() - 0.0) > 1e-12) {
        std::cerr << "2R q=(0,0) position mismatch: (" << p0.x() << ", " << p0.y() << ", " << p0.z() << ")\n";
        return false;
    }

    // q = (π/2, -π/2)
    kin::FKResult res1 = kin::forward_kinematics(model, {M_PI / 2.0, -M_PI / 2.0});
    Eigen::Vector3d p1 = res1.tcp.translation();
    if (std::abs(p1.x() - 1.0) > 1e-12 || std::abs(p1.y() - 1.0) > 1e-12 || std::abs(p1.z() - 0.0) > 1e-12) {
        std::cerr << "2R q=(pi/2, -pi/2) position mismatch: (" << p1.x() << ", " << p1.y() << ", " << p1.z() << ")\n";
        return false;
    }

    std::cout << "PASS: test_2r_planar\n";
    return true;
}

// 3. 3R planar closed form: a=(1, 0.8, 0.5), 100 deterministic q samples
bool test_3r_planar_random() {
    urm::RobotModel model;
    model.links.push_back(urm::Link{"base_link"});

    const double a_vals[3] = {1.0, 0.8, 0.5};
    for (int i = 0; i < 3; ++i) {
        urm::Joint j;
        j.name = "j" + std::to_string(i + 1);
        j.type = urm::JointType::Revolute;
        j.dh.a = a_vals[i];
        model.joints.push_back(j);
        model.links.push_back(urm::Link{"link" + std::to_string(i + 1)});
    }

    std::mt19937 gen(42);
    std::uniform_real_distribution<double> dist(-M_PI, M_PI);

    for (int sample = 0; sample < 100; ++sample) {
        double q1 = dist(gen);
        double q2 = dist(gen);
        double q3 = dist(gen);

        kin::FKResult res = kin::forward_kinematics(model, {q1, q2, q3});

        double th1 = q1;
        double th2 = q1 + q2;
        double th3 = q1 + q2 + q3;

        double x_ref = a_vals[0] * std::cos(th1) + a_vals[1] * std::cos(th2) + a_vals[2] * std::cos(th3);
        double y_ref = a_vals[0] * std::sin(th1) + a_vals[1] * std::sin(th2) + a_vals[2] * std::sin(th3);

        Eigen::Vector3d p = res.tcp.translation();
        if (std::abs(p.x() - x_ref) > 1e-12 || std::abs(p.y() - y_ref) > 1e-12 || std::abs(p.z() - 0.0) > 1e-12) {
            std::cerr << "3R sample " << sample << " position mismatch: got ("
                      << p.x() << ", " << p.y() << "), expected (" << x_ref << ", " << y_ref << ")\n";
            return false;
        }

        double yaw_ref = wrap_to_pi(th3);
        double yaw_act = wrap_to_pi(std::atan2(res.tcp.linear()(1, 0), res.tcp.linear()(0, 0)));
        if (std::abs(yaw_act - yaw_ref) > 1e-12) {
            std::cerr << "3R sample " << sample << " yaw mismatch: got " << yaw_act << ", expected " << yaw_ref << "\n";
            return false;
        }
    }

    std::cout << "PASS: test_3r_planar_random\n";
    return true;
}

// 4. Prismatic: d=0.2, q=0.3 → TCP z = 0.5
bool test_prismatic() {
    urm::RobotModel model;
    model.links.push_back(urm::Link{"base_link"});
    urm::Joint j;
    j.name = "prism1";
    j.type = urm::JointType::Prismatic;
    j.dh.d = 0.2;
    j.q = 0.3;
    model.joints.push_back(j);
    model.links.push_back(urm::Link{"link1"});

    kin::FKResult res = kin::forward_kinematics(model);
    Eigen::Vector3d p = res.tcp.translation();
    if (std::abs(p.x() - 0.0) > 1e-12 || std::abs(p.y() - 0.0) > 1e-12 || std::abs(p.z() - 0.5) > 1e-12) {
        std::cerr << "Prismatic position mismatch: (" << p.x() << ", " << p.y() << ", " << p.z() << ")\n";
        return false;
    }

    std::cout << "PASS: test_prismatic\n";
    return true;
}

// 4b. Prismatic under MDH: d=0.2, q=0.3 → TCP z = 0.5
bool test_prismatic_mdh() {
    urm::RobotModel model;
    model.convention = urm::DHConvention::Modified;
    model.links.push_back(urm::Link{"base_link"});
    urm::Joint j;
    j.name = "prism1";
    j.type = urm::JointType::Prismatic;
    j.mdh.d = 0.2;
    j.q = 0.3;
    model.joints.push_back(j);
    model.links.push_back(urm::Link{"link1"});

    kin::FKResult res = kin::forward_kinematics(model);
    Eigen::Vector3d p = res.tcp.translation();
    if (std::abs(p.x() - 0.0) > 1e-12 || std::abs(p.y() - 0.0) > 1e-12 || std::abs(p.z() - 0.5) > 1e-12) {
        std::cerr << "Prismatic MDH position mismatch: (" << p.x() << ", " << p.y() << ", " << p.z() << ")\n";
        return false;
    }

    std::cout << "PASS: test_prismatic_mdh\n";
    return true;
}

// 4c. Known-answer 3D case, standard DH:
// 2-link arm, link1: a=0, alpha=π/2, d=0.5; link2: a=0.4, alpha=0, d=0; q=(π/2, 0).
// Derivation by hand:
// Standard DH transform A = Rz(θ) · Tz(d) · Tx(a) · Rx(α)
// Joint 1: θ=π/2, d=0.5, a=0, α=π/2
//   Rz(π/2) rotates x->y, y->-x, z->z.
//   Tz(0.5) translates along z by 0.5.
//   Rx(π/2) rotates y->z, z->-y.
//   A1 = [  0  0  1    0  ]
//        [  1  0  0    0  ]
//        [  0  1  0   0.5 ]
//        [  0  0  0    1  ]
// Joint 2: θ=0, d=0, a=0.4, α=0
//   A2 = [  1  0  0   0.4 ]
//        [  0  1  0    0  ]
//        [  0  0  1    0  ]
//        [  0  0  0    1  ]
// Overall TCP = A1 · A2 =
//   [ 0  0  1   0.0 ]
//   [ 1  0  0   0.4 ]
//   [ 0  1  0   0.5 ]
//   [ 0  0  0   1.0 ]
// Expected TCP translation = (0.0, 0.4, 0.5).
// Hard-coded expected matrix; does NOT call dh_transform or mdh_transform.
bool test_3d_known_answer_standard_dh() {
    urm::RobotModel model;
    model.convention = urm::DHConvention::Standard;
    model.links.push_back(urm::Link{"base_link"});

    urm::Joint j1;
    j1.name = "j1";
    j1.type = urm::JointType::Revolute;
    j1.dh.a = 0.0;
    j1.dh.alpha = M_PI / 2.0;
    j1.dh.d = 0.5;
    j1.q = M_PI / 2.0;
    model.joints.push_back(j1);
    model.links.push_back(urm::Link{"link1"});

    urm::Joint j2;
    j2.name = "j2";
    j2.type = urm::JointType::Revolute;
    j2.dh.a = 0.4;
    j2.dh.alpha = 0.0;
    j2.dh.d = 0.0;
    j2.q = 0.0;
    model.joints.push_back(j2);
    model.links.push_back(urm::Link{"link2"});

    kin::FKResult res = kin::forward_kinematics(model);

    Eigen::Matrix4d expected;
    expected << 0.0, 0.0, 1.0, 0.0,
                1.0, 0.0, 0.0, 0.4,
                0.0, 1.0, 0.0, 0.5,
                0.0, 0.0, 0.0, 1.0;

    double diff = (res.tcp.matrix() - expected).norm();
    if (diff > 1e-12) {
        std::cerr << "3D standard DH known-answer mismatch. Diff norm: " << diff << "\n"
                  << "Got:\n" << res.tcp.matrix() << "\nExpected:\n" << expected << "\n";
        return false;
    }
    std::cout << "PASS: test_3d_known_answer_standard_dh\n";
    return true;
}

// 4d. Known-answer 3D case, modified DH (Craig):
// Arm specification: j1 α=0, a=0, d=0.5; j2 α=π/2, a=0, d=0; tool xyz=(0.4, 0, 0); q=(π/2, 0).
// Craig convention: A = Rx(α_{i-1}) · Tx(a_{i-1}) · Rz(θ_i) · Tz(d_i)
// Joint 1: α_0=0, a_0=0, θ_1=π/2, d_1=0.5
//   A1 = Rz(π/2) · Tz(0.5)
//   A1 = [  0 -1  0    0  ]
//        [  1  0  0    0  ]
//        [  0  0  1   0.5 ]
//        [  0  0  0    1  ]
// Joint 2: α_1=π/2, a_1=0, θ_2=0, d_2=0
//   A2 = Rx(π/2) = [ 1  0  0  0 ]
//                  [ 0  0 -1  0 ]
//                  [ 0  1  0  0 ]
//                  [ 0  0  0  1 ]
// Frame 2 = A1 · A2 =
//   [ 0  0  1   0.0 ]
//   [ 1  0  0   0.0 ]
//   [ 0  1  0   0.5 ]
//   [ 0  0  0   1.0 ]
// Tool offset: xyz = (0.4, 0, 0)
//   TCP = Frame 2 · T_tool
//   TCP col 3 = Frame 2 * [0.4, 0, 0, 1]^T = (0.0, 0.4, 0.5)
// Overall expected TCP matrix:
//   [ 0  0  1   0.0 ]
//   [ 1  0  0   0.4 ]
//   [ 0  1  0   0.5 ]
//   [ 0  0  0   1.0 ]
// Hard-coded expected matrix; does NOT call dh_transform or mdh_transform.
bool test_3d_known_answer_mdh() {
    urm::RobotModel model;
    model.convention = urm::DHConvention::Modified;
    model.links.push_back(urm::Link{"base_link"});

    urm::Joint j1;
    j1.name = "j1";
    j1.type = urm::JointType::Revolute;
    j1.mdh.alpha = 0.0;
    j1.mdh.a = 0.0;
    j1.mdh.d = 0.5;
    j1.q = M_PI / 2.0;
    model.joints.push_back(j1);
    model.links.push_back(urm::Link{"link1"});

    urm::Joint j2;
    j2.name = "j2";
    j2.type = urm::JointType::Revolute;
    j2.mdh.alpha = M_PI / 2.0;
    j2.mdh.a = 0.0;
    j2.mdh.d = 0.0;
    j2.q = 0.0;
    model.joints.push_back(j2);
    model.links.push_back(urm::Link{"link2"});

    model.tool.xyz = {0.4, 0.0, 0.0};

    kin::FKResult res = kin::forward_kinematics(model);

    Eigen::Matrix4d expected;
    expected << 0.0, 0.0, 1.0, 0.0,
                1.0, 0.0, 0.0, 0.4,
                0.0, 1.0, 0.0, 0.5,
                0.0, 0.0, 0.0, 1.0;

    double diff = (res.tcp.matrix() - expected).norm();
    if (diff > 1e-12) {
        std::cerr << "3D MDH known-answer mismatch. Diff norm: " << diff << "\n"
                  << "Got:\n" << res.tcp.matrix() << "\nExpected:\n" << expected << "\n";
        return false;
    }
    std::cout << "PASS: test_3d_known_answer_mdh\n";
    return true;
}

// 5. Fixed joint: contributes transform, dof() unchanged, q ignored
bool test_fixed_joint() {
    urm::RobotModel model;
    model.links.push_back(urm::Link{"base_link"});

    urm::Joint j1;
    j1.name = "j1";
    j1.type = urm::JointType::Revolute;
    j1.dh.a = 1.0;
    j1.q = 0.0;
    model.joints.push_back(j1);
    model.links.push_back(urm::Link{"link1"});

    urm::Joint j_fixed;
    j_fixed.name = "fixed_ext";
    j_fixed.type = urm::JointType::Fixed;
    j_fixed.dh.a = 0.5;
    j_fixed.q = 99.0; // should be ignored
    model.joints.push_back(j_fixed);
    model.links.push_back(urm::Link{"link2"});

    if (model.dof() != 1) {
        std::cerr << "Expected DOF == 1\n";
        return false;
    }

    kin::FKResult res1 = kin::forward_kinematics(model);
    Eigen::Vector3d p1 = res1.tcp.translation();
    if (std::abs(p1.x() - 1.5) > 1e-12 || std::abs(p1.y() - 0.0) > 1e-12) {
        std::cerr << "Fixed joint position mismatch with q=99: (" << p1.x() << ", " << p1.y() << ")\n";
        return false;
    }

    // Overload with explicit q: only 1 value needed
    kin::FKResult res2 = kin::forward_kinematics(model, {0.0});
    Eigen::Vector3d p2 = res2.tcp.translation();
    if (std::abs(p2.x() - 1.5) > 1e-12 || std::abs(p2.y() - 0.0) > 1e-12) {
        std::cerr << "Fixed joint position mismatch with overload: (" << p2.x() << ", " << p2.y() << ")\n";
        return false;
    }

    std::cout << "PASS: test_fixed_joint\n";
    return true;
}

// 6. DH vs MDH equivalence: same 2R planar arm
bool test_dh_vs_mdh() {
    // Standard DH 2R
    urm::RobotModel dh_bot;
    dh_bot.convention = urm::DHConvention::Standard;
    dh_bot.links.push_back(urm::Link{"base"});
    for (int i = 0; i < 2; ++i) {
        urm::Joint j;
        j.name = "j" + std::to_string(i + 1);
        j.type = urm::JointType::Revolute;
        j.dh.a = 1.0;
        dh_bot.joints.push_back(j);
        dh_bot.links.push_back(urm::Link{"link" + std::to_string(i + 1)});
    }

    // Craig Modified DH 2R
    // In Craig MDH: j1 has a0=0, alpha0=0; j2 has a1=1, alpha1=0.
    // Tool transform holds the length of the final link: tool xyz = (1, 0, 0).
    urm::RobotModel mdh_bot;
    mdh_bot.convention = urm::DHConvention::Modified;
    mdh_bot.links.push_back(urm::Link{"base"});

    urm::Joint mj1;
    mj1.name = "mj1";
    mj1.type = urm::JointType::Revolute;
    mj1.mdh.a = 0.0;
    mdh_bot.joints.push_back(mj1);
    mdh_bot.links.push_back(urm::Link{"link1"});

    urm::Joint mj2;
    mj2.name = "mj2";
    mj2.type = urm::JointType::Revolute;
    mj2.mdh.a = 1.0;
    mdh_bot.joints.push_back(mj2);
    mdh_bot.links.push_back(urm::Link{"link2"});

    mdh_bot.tool.xyz = {1.0, 0.0, 0.0};

    std::mt19937 gen(101);
    std::uniform_real_distribution<double> dist(-M_PI, M_PI);

    for (int sample = 0; sample < 20; ++sample) {
        double q1 = dist(gen);
        double q2 = dist(gen);

        kin::FKResult r_dh = kin::forward_kinematics(dh_bot, {q1, q2});
        kin::FKResult r_mdh = kin::forward_kinematics(mdh_bot, {q1, q2});

        if ((r_dh.tcp.translation() - r_mdh.tcp.translation()).norm() > 1e-12) {
            std::cerr << "DH vs MDH position mismatch at sample " << sample << "\n";
            return false;
        }
        if ((r_dh.tcp.linear() - r_mdh.tcp.linear()).norm() > 1e-12) {
            std::cerr << "DH vs MDH orientation mismatch at sample " << sample << "\n";
            return false;
        }
    }

    std::cout << "PASS: test_dh_vs_mdh\n";
    return true;
}

// 7. Base + tool: base yaw rotates TCP; tool xyz adds along last frame
bool test_base_and_tool() {
    urm::RobotModel model;
    model.base.rpy = {0.0, 0.0, M_PI / 2.0}; // 90 deg yaw
    model.tool.xyz = {0.2, 0.0, 0.0};       // 0.2 along end effector x

    model.links.push_back(urm::Link{"base_link"});
    urm::Joint j;
    j.name = "j1";
    j.type = urm::JointType::Revolute;
    j.dh.a = 1.0;
    j.q = 0.0;
    model.joints.push_back(j);
    model.links.push_back(urm::Link{"link1"});

    kin::FKResult res = kin::forward_kinematics(model);
    Eigen::Vector3d p = res.tcp.translation();
    // Arm points along arm x (length 1.0) + tool offset 0.2 = 1.2 total along arm axis.
    // Base yaw rotates arm axis into world +y: so TCP = (0, 1.2, 0).
    if (std::abs(p.x() - 0.0) > 1e-12 || std::abs(p.y() - 1.2) > 1e-12 || std::abs(p.z() - 0.0) > 1e-12) {
        std::cerr << "Base+Tool position mismatch: (" << p.x() << ", " << p.y() << ", " << p.z() << ")\n";
        return false;
    }

    std::cout << "PASS: test_base_and_tool\n";
    return true;
}

// 8. pose_to_iso rotation order
bool test_pose_to_iso_order() {
    urm::Pose pose;
    pose.xyz = {1.2, -3.4, 5.6};
    pose.rpy = {0.3, -0.7, 1.1};

    Eigen::Isometry3d iso = kin::pose_to_iso(pose);

    Eigen::AngleAxisd roll(0.3, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitch(-0.7, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yaw(1.1, Eigen::Vector3d::UnitZ());
    Eigen::Matrix3d R_expected = (yaw * pitch * roll).toRotationMatrix();

    if ((iso.linear() - R_expected).norm() > 1e-12) {
        std::cerr << "pose_to_iso rotation mismatch\n";
        return false;
    }
    if ((iso.translation() - Eigen::Vector3d(1.2, -3.4, 5.6)).norm() > 1e-12) {
        std::cerr << "pose_to_iso translation mismatch\n";
        return false;
    }

    std::cout << "PASS: test_pose_to_iso_order\n";
    return true;
}

// 9. Orthonormality and determinant = +1
bool test_orthonormality() {
    urm::RobotModel model;
    model.base.rpy = {0.1, -0.2, 0.3};
    model.tool.rpy = {0.4, 0.5, -0.6};
    model.links.push_back(urm::Link{"base"});
    for (int i = 0; i < 3; ++i) {
        urm::Joint j;
        j.name = "j" + std::to_string(i + 1);
        j.type = urm::JointType::Revolute;
        j.dh.a = 0.5;
        j.dh.alpha = 0.4;
        j.dh.d = 0.3;
        j.q = 0.2 * (i + 1);
        model.joints.push_back(j);
        model.links.push_back(urm::Link{"link" + std::to_string(i + 1)});
    }

    kin::FKResult res = kin::forward_kinematics(model);

    auto check_rot = [](const Eigen::Matrix3d& R) -> bool {
        Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
        if ((R * R.transpose() - I).norm() > 1e-12) return false;
        if (std::abs(R.determinant() - 1.0) > 1e-12) return false;
        return true;
    };

    for (size_t i = 0; i < res.frames.size(); ++i) {
        if (!check_rot(res.frames[i].linear())) {
            std::cerr << "Frame " << i << " rotation not orthonormal with det=+1\n";
            return false;
        }
    }
    if (!check_rot(res.tcp.linear())) {
        std::cerr << "TCP rotation not orthonormal with det=+1\n";
        return false;
    }

    std::cout << "PASS: test_orthonormality\n";
    return true;
}

// 10. Frame count, wrong-size q throws, overload constness
bool test_fk_api_contracts() {
    urm::RobotModel model;
    model.links.push_back(urm::Link{"base"});
    for (int i = 0; i < 2; ++i) {
        urm::Joint j;
        j.name = "j" + std::to_string(i + 1);
        j.type = urm::JointType::Revolute;
        j.dh.a = 1.0;
        j.q = 0.5 * (i + 1);
        model.joints.push_back(j);
        model.links.push_back(urm::Link{"link" + std::to_string(i + 1)});
    }

    kin::FKResult res = kin::forward_kinematics(model);
    if (res.frames.size() != 3) {
        std::cerr << "Expected 3 frames for 2 joints, got " << res.frames.size() << "\n";
        return false;
    }

    // Wrong-size q throws (too short and too long)
    bool threw_short = false;
    try {
        kin::forward_kinematics(model, {0.1});
    } catch (const std::invalid_argument&) {
        threw_short = true;
    }
    if (!threw_short) {
        std::cerr << "Expected invalid_argument on 1 q for 2 DOF model (too short)\n";
        return false;
    }

    bool threw_long = false;
    try {
        kin::forward_kinematics(model, {0.1, 0.2, 0.3});
    } catch (const std::invalid_argument&) {
        threw_long = true;
    }
    if (!threw_long) {
        std::cerr << "Expected invalid_argument on 3 q for 2 DOF model (too long)\n";
        return false;
    }

    // Overload leaves model.joints[i].q unchanged
    double q1_orig = model.joints[0].q;
    double q2_orig = model.joints[1].q;
    kin::forward_kinematics(model, {1.23, 4.56});
    if (model.joints[0].q != q1_orig || model.joints[1].q != q2_orig) {
        std::cerr << "Overload forward_kinematics mutated model joint q\n";
        return false;
    }

    std::cout << "PASS: test_fk_api_contracts\n";
    return true;
}

int main() {
    int failures = 0;
    if (!test_1r())                    { ++failures; }
    if (!test_2r_planar())             { ++failures; }
    if (!test_3r_planar_random())      { ++failures; }
    if (!test_prismatic())             { ++failures; }
    if (!test_prismatic_mdh())         { ++failures; }
    if (!test_3d_known_answer_standard_dh()) { ++failures; }
    if (!test_3d_known_answer_mdh())   { ++failures; }
    if (!test_fixed_joint())           { ++failures; }
    if (!test_dh_vs_mdh())             { ++failures; }
    if (!test_base_and_tool())         { ++failures; }
    if (!test_pose_to_iso_order())     { ++failures; }
    if (!test_orthonormality())        { ++failures; }
    if (!test_fk_api_contracts())      { ++failures; }

    if (failures == 0) {
        std::cout << "\nAll FK C++ unit tests passed.\n";
        return 0;
    } else {
        std::cerr << "\n" << failures << " FK test(s) FAILED.\n";
        return 1;
    }
}

