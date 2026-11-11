/**
 * test_eigen.cpp
 * R0.5: Verifies Eigen 3.4.0 builds and produces correct linear algebra results.
 *
 * Tests:
 *   1. 4x4 identity matrix properties.
 *   2. 3D rotation matrix (90 deg around Z-axis).
 *   3. Homogeneous transform composition.
 *   4. Transform inverse.
 */

#define _USE_MATH_DEFINES
#include <iostream>
#include <cmath>
#include <Eigen/Dense>

static const double TOL = 1e-10;

bool nearly_equal(double a, double b, double tol = TOL) {
    return std::abs(a - b) < tol;
}

bool matrix_nearly_equal(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B, double tol = TOL) {
    return (A - B).cwiseAbs().maxCoeff() < tol;
}

// Test 1: 4x4 identity matrix
bool test_identity() {
    Eigen::Matrix4d I = Eigen::Matrix4d::Identity();

    // Diagonal must be 1
    for (int i = 0; i < 4; ++i) {
        if (!nearly_equal(I(i, i), 1.0)) return false;
    }
    // Off-diagonal must be 0
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            if (i != j && !nearly_equal(I(i, j), 0.0)) return false;
        }
    }
    // det(I) == 1
    if (!nearly_equal(I.determinant(), 1.0)) return false;

    std::cout << "PASS: test_identity\n";
    return true;
}

// Test 2: Rotation matrix — 90 degrees around Z-axis
bool test_rotation_z90() {
    // R_z(90) = [[0, -1, 0], [1, 0, 0], [0, 0, 1]]
    Eigen::AngleAxisd rot(M_PI / 2.0, Eigen::Vector3d::UnitZ());
    Eigen::Matrix3d R = rot.toRotationMatrix();

    if (!nearly_equal(R(0, 0),  0.0, 1e-9)) return false;
    if (!nearly_equal(R(0, 1), -1.0, 1e-9)) return false;
    if (!nearly_equal(R(1, 0),  1.0, 1e-9)) return false;
    if (!nearly_equal(R(1, 1),  0.0, 1e-9)) return false;
    if (!nearly_equal(R(2, 2),  1.0, 1e-9)) return false;

    // det(R) == 1 (rotation, not reflection)
    if (!nearly_equal(R.determinant(), 1.0, 1e-9)) return false;

    // R^T * R == I (orthogonal)
    Eigen::Matrix3d should_be_I = R.transpose() * R;
    for (int i = 0; i < 3; ++i) {
        if (!nearly_equal(should_be_I(i, i), 1.0, 1e-9)) return false;
    }

    std::cout << "PASS: test_rotation_z90\n";
    return true;
}

// Test 3: Homogeneous transform composition
bool test_transform_composition() {
    // T1: rotate 90 deg around Z, no translation
    Eigen::Isometry3d T1 = Eigen::Isometry3d::Identity();
    T1.rotate(Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitZ()));

    // T2: no rotation, translate (1, 0, 0)
    Eigen::Isometry3d T2 = Eigen::Isometry3d::Identity();
    T2.translate(Eigen::Vector3d(1.0, 0.0, 0.0));

    // T1 * T2: rotate first, then translate in rotated frame
    // Translation (1,0,0) in world becomes (0,1,0) after 90-deg Z rotation
    Eigen::Isometry3d T12 = T1 * T2;
    Eigen::Vector3d translation = T12.translation();

    if (!nearly_equal(translation.x(), 0.0, 1e-9)) return false;
    if (!nearly_equal(translation.y(), 1.0, 1e-9)) return false;
    if (!nearly_equal(translation.z(), 0.0, 1e-9)) return false;

    std::cout << "PASS: test_transform_composition\n";
    return true;
}

// Test 4: Transform inverse
bool test_transform_inverse() {
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    T.rotate(Eigen::AngleAxisd(M_PI / 4.0, Eigen::Vector3d::UnitY()));
    T.translate(Eigen::Vector3d(2.0, 3.0, 4.0));

    Eigen::Isometry3d T_inv = T.inverse();
    Eigen::Isometry3d should_be_I = T * T_inv;

    // T * T_inv should be identity
    Eigen::Matrix4d diff = should_be_I.matrix() - Eigen::Matrix4d::Identity();
    if (diff.cwiseAbs().maxCoeff() > 1e-10) return false;

    std::cout << "PASS: test_transform_inverse\n";
    return true;
}

int main() {
    int failures = 0;

    if (!test_identity())             { std::cerr << "FAIL: test_identity\n";             ++failures; }
    if (!test_rotation_z90())         { std::cerr << "FAIL: test_rotation_z90\n";         ++failures; }
    if (!test_transform_composition()){ std::cerr << "FAIL: test_transform_composition\n"; ++failures; }
    if (!test_transform_inverse())    { std::cerr << "FAIL: test_transform_inverse\n";    ++failures; }

    if (failures == 0) {
        std::cout << "\nAll Eigen tests passed.\n";
        return 0;
    } else {
        std::cerr << "\n" << failures << " Eigen test(s) FAILED.\n";
        return 1;
    }
}
