/**
 * test_compiled_arrays.cpp
 * R4.1: compiled layout freeze. to_arrays / from_arrays round trip and checks.
 */
#include <algorithm>
#include <cmath>
#include <functional>
#include <iostream>
#include <stdexcept>
#include <string>

#include "kin/compiled_arrays.hpp"
#include "kin/compiled_model.hpp"
#include "kin/fk.hpp"
#include "urm/robot_model.hpp"

using urm::Joint; using urm::JointType; using urm::JointKinematics;

static int g_fail = 0;
#define CHECK(c) do { if (!(c)) { std::cerr << "FAIL line " << __LINE__ << ": " #c "\n"; ++g_fail; } } while (0)

static bool throws_invalid(const std::function<void()>& f) {
    try { f(); } catch (const std::invalid_argument&) { return true; } catch (...) { return false; }
    return false;
}

static Joint oa(const std::string& name, int p, int c, JointType t,
                std::array<double, 3> xyz, std::array<double, 3> axis) {
    Joint j; j.name = name; j.parent_link = p; j.child_link = c; j.type = t;
    j.kinematics = JointKinematics::OriginAxis;
    j.origin.xyz = xyz; j.axis = axis;
    return j;
}

static urm::RobotModel make_links(int n) {
    urm::RobotModel m;
    for (int i = 0; i < n; ++i) m.links.push_back(urm::Link{"l" + std::to_string(i)});
    return m;
}

// Max abs difference between all frames and tcp of two FK results.
static double max_diff(const kin::FKResult& a, const kin::FKResult& b) {
    double d = 0.0;
    if (a.frames.size() != b.frames.size()) return 1e300;
    for (size_t k = 0; k < a.frames.size(); ++k)
        d = std::max(d, (a.frames[k].matrix() - b.frames[k].matrix()).cwiseAbs().maxCoeff());
    d = std::max(d, (a.tcp.matrix() - b.tcp.matrix()).cwiseAbs().maxCoeff());
    return d;
}

static void check_round_trip(const urm::RobotModel& m, const std::vector<double>& q) {
    const kin::CompiledModel cm = kin::compile(m);
    const kin::CompiledArrays arr = kin::to_arrays(cm);
    const kin::CompiledModel back = kin::from_arrays(arr);
    CHECK(back.joints.size() == cm.joints.size());
    CHECK(back.n_links == cm.n_links);
    CHECK(back.n_dof == cm.n_dof);
    CHECK(back.tcp_link == cm.tcp_link);
    const double d = max_diff(kin::forward_kinematics(cm, q), kin::forward_kinematics(back, q));
    if (!(d <= 1e-15)) std::cerr << "round trip diff " << d << "\n";
    CHECK(d <= 1e-15);
}

// DH robot: 3 revolute, standard convention.
static void test_round_trip_dh() {
    urm::RobotModel m = make_links(4);
    m.convention = urm::DHConvention::Standard;
    double a[3] = {0.4, 0.3, 0.2}, al[3] = {0.5, -0.7, 0.0}, d[3] = {0.1, 0.0, 0.05};
    for (int i = 0; i < 3; ++i) {
        Joint j; j.name = "j" + std::to_string(i);
        j.parent_link = i; j.child_link = i + 1; j.type = JointType::Revolute;
        j.dh = {a[i], al[i], d[i], 0.2 * i};
        m.joints.push_back(j);
    }
    check_round_trip(m, {0.3, -0.8, 1.1});
}

// Tree with a branch: link 1 has two children (2 and 3). tcp on link 2.
static void test_round_trip_tree() {
    urm::RobotModel m = make_links(4);
    m.joints.push_back(oa("root", 0, 1, JointType::Revolute, {0, 0, 0.2}, {0, 0, 1}));
    m.joints.push_back(oa("b1", 1, 2, JointType::Revolute, {0.5, 0, 0}, {0, 1, 0}));
    m.joints.push_back(oa("b2", 1, 3, JointType::Prismatic, {0, 0.3, 0}, {1, 0, 0}));
    m.tcp_link = 2;
    check_round_trip(m, {0.7, -0.4, 0.25});
}

// Mimic: j1 follows j0 with multiplier 2 and offset 0.1. Also a fixed joint.
static void test_round_trip_mimic() {
    urm::RobotModel m = make_links(4);
    m.joints.push_back(oa("j0", 0, 1, JointType::Revolute, {0, 0, 0.1}, {0, 0, 1}));
    Joint j1 = oa("j1", 1, 2, JointType::Revolute, {0.2, 0, 0}, {0, 1, 0});
    j1.mimic = urm::Mimic{};
    j1.mimic->joint = "j0";
    j1.mimic->multiplier = 2.0;
    j1.mimic->offset = 0.1;
    m.joints.push_back(j1);
    m.joints.push_back(oa("f", 2, 3, JointType::Fixed, {0.1, 0.1, 0}, {0, 0, 1}));
    m.tcp_link = 3;
    CHECK(m.dof() == 1);
    check_round_trip(m, {0.45});
}

// Layout: enum values and row-major matrix order.
static void test_layout_values() {
    urm::RobotModel m = make_links(3);
    m.joints.push_back(oa("r", 0, 1, JointType::Revolute, {1, 2, 3}, {0, 0, 1}));
    m.joints.push_back(oa("p", 1, 2, JointType::Prismatic, {0, 0, 0}, {1, 0, 0}));
    kin::CompiledArrays a = kin::to_arrays(kin::compile(m));
    CHECK(a.layout_version == 1);
    CHECK(a.n_joints == 2);
    CHECK(a.type.size() == 2);
    CHECK(a.type[0] == 0 && a.type[1] == 1);            // Revolute 0, Prismatic 1
    CHECK(static_cast<int>(JointType::Revolute) == 0);
    CHECK(static_cast<int>(JointType::Prismatic) == 1);
    CHECK(static_cast<int>(JointType::Fixed) == 2);
    CHECK(a.pre.size() == 32 && a.post.size() == 32);
    CHECK(a.axis.size() == 6);
    CHECK(a.mult.size() == 2 && a.offset.size() == 2);
    CHECK(a.base.size() == 16 && a.tool.size() == 16);
    // pre of joint 0 is translation (1,2,3): row-major entries 3, 7, 11 hold x, y, z.
    CHECK(std::fabs(a.pre[3] - 1.0) < 1e-15);
    CHECK(std::fabs(a.pre[7] - 2.0) < 1e-15);
    CHECK(std::fabs(a.pre[11] - 3.0) < 1e-15);
    CHECK(std::fabs(a.pre[15] - 1.0) < 1e-15);
    CHECK(std::fabs(a.pre[12]) < 1e-15 && std::fabs(a.pre[13]) < 1e-15 && std::fabs(a.pre[14]) < 1e-15);
}

static void test_bad_arrays_throw() {
    urm::RobotModel m = make_links(2);
    m.joints.push_back(oa("r", 0, 1, JointType::Revolute, {0, 0, 0}, {0, 0, 1}));
    const kin::CompiledArrays good = kin::to_arrays(kin::compile(m));

    kin::CompiledArrays bad = good;
    bad.layout_version = 2;
    CHECK(throws_invalid([&] { kin::from_arrays(bad); }));

    bad = good; bad.pre.pop_back();
    CHECK(throws_invalid([&] { kin::from_arrays(bad); }));

    bad = good; bad.axis.push_back(0.0);
    CHECK(throws_invalid([&] { kin::from_arrays(bad); }));

    bad = good; bad.mult.clear();
    CHECK(throws_invalid([&] { kin::from_arrays(bad); }));

    bad = good; bad.base.resize(15);
    CHECK(throws_invalid([&] { kin::from_arrays(bad); }));

    bad = good; bad.type[0] = 7;
    CHECK(throws_invalid([&] { kin::from_arrays(bad); }));

    // The good one still works.
    CHECK(!throws_invalid([&] { kin::from_arrays(good); }));
}

static void test_effective_links_legacy() {
    urm::RobotModel m = make_links(3);
    for (int i = 0; i < 2; ++i) {
        Joint j; j.name = "j" + std::to_string(i);
        j.parent_link = 0; j.child_link = 0; j.type = JointType::Revolute;
        m.joints.push_back(j);
    }
    auto eff = kin::effective_links(m);
    CHECK(eff.size() == 2);
    CHECK(eff[0].first == 0 && eff[0].second == 1);
    CHECK(eff[1].first == 1 && eff[1].second == 2);
}

int main() {
    test_round_trip_dh();
    test_round_trip_tree();
    test_round_trip_mimic();
    test_layout_values();
    test_bad_arrays_throw();
    test_effective_links_legacy();
    if (g_fail == 0) std::cout << "test_compiled_arrays: all passed\n";
    return g_fail == 0 ? 0 : 1;
}
