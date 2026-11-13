/**
 * test_compiled.cpp
 * R3.2: compiled kinematic model + tree FK tests.
 */
#include <iostream>
#include <random>
#include <stdexcept>
#include <cmath>
#include <functional>

#include "kin/fk.hpp"
#include "kin/compiled_model.hpp"
#include "urm/robot_model.hpp"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using Iso = Eigen::Isometry3d;
using urm::Joint; using urm::JointType; using urm::JointKinematics;

static int g_fail = 0;
#define CHECK(c) do { if (!(c)) { std::cerr << "FAIL line " << __LINE__ << ": " #c "\n"; ++g_fail; } } while (0)

static bool throws(const std::function<void()>& f) {
    try { f(); } catch (const std::invalid_argument&) { return true; }
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

// Textbook A matrices, written out by hand (no dh_transform call).
static Iso textbook_std(double a, double al, double d, double th) {
    Eigen::Matrix4d M;
    double ct = std::cos(th), st = std::sin(th), ca = std::cos(al), sa = std::sin(al);
    M << ct, -st * ca,  st * sa, a * ct,
         st,  ct * ca, -ct * sa, a * st,
         0,  sa,        ca,      d,
         0,  0,         0,       1;
    Iso I; I.matrix() = M; return I;
}
static Iso textbook_mod(double a, double al, double d, double th) {
    Eigen::Matrix4d M;
    double ct = std::cos(th), st = std::sin(th), ca = std::cos(al), sa = std::sin(al);
    M << ct,       -st,       0,   a,
         st * ca,   ct * ca, -sa, -sa * d,
         st * sa,   ct * sa,  ca,  ca * d,
         0, 0, 0, 1;
    Iso I; I.matrix() = M; return I;
}

static void test_random_chains() {
    std::mt19937 rng(42);
    std::uniform_real_distribution<double> U(-1.5, 1.5);
    for (int t = 0; t < 200; ++t) {
        int n = 3 + static_cast<int>(rng() % 5);
        urm::RobotModel m = make_links(n + 1);
        m.convention = (t % 2) ? urm::DHConvention::Modified : urm::DHConvention::Standard;
        std::vector<double> q;
        for (int i = 0; i < n; ++i) {
            Joint j; j.name = "j" + std::to_string(i);
            j.parent_link = i; j.child_link = i + 1;
            int ty = static_cast<int>(rng() % 3);
            j.type = ty == 0 ? JointType::Revolute : ty == 1 ? JointType::Prismatic : JointType::Fixed;
            double a = U(rng), al = U(rng), d = U(rng), th = U(rng);
            j.dh = {a, al, d, th}; j.mdh = {a, al, d, th};
            m.joints.push_back(j);
        }
        int nd = m.dof();
        for (int k = 0; k < 5; ++k) {
            q.clear();
            for (int i = 0; i < nd; ++i) q.push_back(U(rng));
            Iso ref = Iso::Identity(); size_t qi = 0;
            for (const auto& j : m.joints) {
                double qv = j.type == JointType::Fixed ? 0.0 : q[qi++];
                double th = j.dh.theta, d = j.dh.d;
                if (j.type == JointType::Revolute) th += qv;
                if (j.type == JointType::Prismatic) d += qv;
                ref = ref * (m.convention == urm::DHConvention::Standard
                                 ? textbook_std(j.dh.a, j.dh.alpha, d, th)
                                 : textbook_mod(j.dh.a, j.dh.alpha, d, th));
            }
            auto res = kin::forward_kinematics(m, q);
            CHECK((res.tcp.matrix() - ref.matrix()).cwiseAbs().maxCoeff() < 1e-12);
        }
    }
}

static void test_origin_axis() {
    urm::RobotModel m = make_links(2);
    m.joints.push_back(oa("j", 0, 1, JointType::Revolute, {1, 0, 0}, {0, 0, 1}));
    m.tool.xyz = {1, 0, 0};
    auto r = kin::forward_kinematics(m, {M_PI / 2});
    CHECK((r.frames[1].translation() - Eigen::Vector3d(1, 0, 0)).norm() < 1e-12);
    CHECK((r.frames[1].linear() - Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ()).toRotationMatrix()).norm() < 1e-12);
    CHECK((r.tcp.translation() - Eigen::Vector3d(1, 1, 0)).norm() < 1e-12);

    // off-axis
    urm::RobotModel m2 = make_links(2);
    double s = 1.0 / std::sqrt(3.0);
    m2.joints.push_back(oa("j", 0, 1, JointType::Revolute, {0, 0, 0}, {1, 1, 1}));
    auto r2 = kin::forward_kinematics(m2, {0.7});
    Eigen::Matrix3d R = Eigen::AngleAxisd(0.7, Eigen::Vector3d(s, s, s)).toRotationMatrix();
    CHECK((r2.frames[1].linear() - R).norm() < 1e-12);

    // prismatic
    urm::RobotModel m3 = make_links(2);
    m3.joints.push_back(oa("j", 0, 1, JointType::Prismatic, {0, 0, 0}, {0, 1, 0}));
    auto r3 = kin::forward_kinematics(m3, {0.3});
    CHECK((r3.frames[1].translation() - Eigen::Vector3d(0, 0.3, 0)).norm() < 1e-12);
}

static urm::RobotModel tree_model() {
    urm::RobotModel m = make_links(4);
    m.joints.push_back(oa("j1", 0, 1, JointType::Revolute, {1, 0, 0}, {0, 0, 1}));
    m.joints.push_back(oa("j2", 0, 2, JointType::Revolute, {0, 1, 0}, {0, 0, 1}));
    m.joints.push_back(oa("j3", 1, 3, JointType::Revolute, {0.5, 0, 0}, {0, 0, 1}));
    return m;
}

static void test_order() {
    urm::RobotModel sorted = make_links(4);
    sorted.joints.push_back(oa("a", 0, 1, JointType::Revolute, {1, 0, 0}, {0, 0, 1}));
    sorted.joints.push_back(oa("b", 0, 2, JointType::Revolute, {0, 1, 0}, {0, 1, 0}));
    sorted.joints.push_back(oa("c", 2, 3, JointType::Revolute, {0.4, 0, 0}, {1, 0, 0}));
    urm::RobotModel uns = make_links(4);
    uns.joints.push_back(sorted.joints[2]);
    uns.joints.push_back(sorted.joints[1]);
    uns.joints.push_back(sorted.joints[0]);
    // q order follows joint list: sorted = (a,b,c); unsorted = (c,b,a)
    auto r1 = kin::forward_kinematics(sorted, {0.3, 0.5, 0.7});
    auto r2 = kin::forward_kinematics(uns, {0.7, 0.5, 0.3});
    for (int i = 0; i < 4; ++i)
        CHECK((r1.frames[i].matrix() - r2.frames[i].matrix()).norm() < 1e-12);
}

static void test_mimic() {
    urm::RobotModel m = make_links(4);
    m.joints.push_back(oa("A", 0, 1, JointType::Revolute, {0, 0, 0}, {0, 0, 1}));
    m.joints.push_back(oa("B", 1, 2, JointType::Revolute, {0, 0, 0}, {0, 0, 1}));
    m.joints.push_back(oa("C", 2, 3, JointType::Revolute, {0, 0, 0}, {0, 0, 1}));
    m.joints[1].mimic = urm::Mimic{"A", 2.0, 0.1};
    m.joints[2].mimic = urm::Mimic{"B", -1.0, 0.2};
    CHECK(m.dof() == 1);
    auto cm = kin::compile(m);
    CHECK(std::abs(cm.joints[2].mult - (-2.0)) < 1e-15);
    CHECK(std::abs(cm.joints[2].offset - (-1.0 * 0.1 + 0.2)) < 1e-15);  // m2*o1 + o2 = -0.1 + 0.2
    // C q_eff = -2 qA + 0.1 ; total rotation = qA + (2qA+0.1) + (-2qA+0.1) = qA + 0.2
    double qa = 0.4;
    auto r = kin::forward_kinematics(cm, {qa});
    double total = std::atan2(r.frames[3](1, 0), r.frames[3](0, 0));
    CHECK(std::abs(total - (qa + 0.2)) < 1e-12);

    // master q=0.4 mult=-1 off=0.1 -> -0.3
    urm::RobotModel m2 = make_links(3);
    m2.joints.push_back(oa("M", 0, 1, JointType::Revolute, {0, 0, 0}, {0, 0, 1}));
    m2.joints.push_back(oa("S", 1, 2, JointType::Revolute, {0, 0, 0}, {0, 0, 1}));
    m2.joints[1].mimic = urm::Mimic{"M", -1.0, 0.1};
    CHECK(m2.dof() == 1 && m2.get_q().size() == 1);
    auto cm2 = kin::compile(m2);
    CHECK(std::abs(cm2.joints[1].mult * 0.4 + cm2.joints[1].offset - (-0.3)) < 1e-12);

    // mimic of fixed joint
    urm::RobotModel m3 = make_links(3);
    m3.joints.push_back(oa("F", 0, 1, JointType::Fixed, {0, 0, 0}, {0, 0, 1}));
    m3.joints.push_back(oa("S", 1, 2, JointType::Revolute, {0, 0, 0}, {0, 0, 1}));
    m3.joints[1].mimic = urm::Mimic{"F", 1.0, 0.0};
    CHECK(throws([&] { kin::compile(m3); }));
    bool reported = false;
    for (auto& e : m3.validate()) if (e.find("fixed joint") != std::string::npos) reported = true;
    CHECK(reported);
    // valid model: validate reports nothing about fixed
    for (auto& e : m.validate()) CHECK(e.find("fixed joint") == std::string::npos);
    // cycle
    urm::RobotModel m4 = make_links(3);
    m4.joints.push_back(oa("X", 0, 1, JointType::Revolute, {0, 0, 0}, {0, 0, 1}));
    m4.joints.push_back(oa("Y", 1, 2, JointType::Revolute, {0, 0, 0}, {0, 0, 1}));
    m4.joints[0].mimic = urm::Mimic{"Y", 1, 0};
    m4.joints[1].mimic = urm::Mimic{"X", 1, 0};
    CHECK(throws([&] { kin::compile(m4); }));
    // missing target
    m4.joints[0].mimic = urm::Mimic{"nope", 1, 0};
    m4.joints[1].mimic.reset();
    CHECK(throws([&] { kin::compile(m4); }));
}

static void test_tree() {
    urm::RobotModel m = tree_model();
    m.tcp_link = 3;
    auto r = kin::forward_kinematics(m, {M_PI / 2, 0, 0});
    CHECK((r.frames[3].translation() - Eigen::Vector3d(1, 0.5, 0)).norm() < 1e-12);
    CHECK((r.tcp.translation() - r.frames[3].translation()).norm() < 1e-12);
    auto c3 = kin::active_chain(m);
    CHECK((c3 == std::vector<int>{0, 2}));
    m.tcp_link = 2;
    auto r2 = kin::forward_kinematics(m, {M_PI / 2, 0, 0});
    CHECK((r2.tcp.translation() - Eigen::Vector3d(0, 1, 0)).norm() < 1e-12);
    CHECK((r.tcp.translation() - r2.tcp.translation()).norm() > 0.1);
    auto c2 = kin::active_chain(m);
    CHECK((c2 == std::vector<int>{1}));
    m.tcp_link = -1;  // last link = 3
    CHECK((kin::active_chain(m) == std::vector<int>{0, 2}));
}

static void test_misc() {
    urm::RobotModel m = tree_model();
    m.joints[1].type = JointType::Fixed;
    CHECK(m.dof() == 2);
    auto a = kin::forward_kinematics(m, {0.1, 0.2});
    CHECK(a.frames.size() == 4);
    CHECK(throws([&] { kin::forward_kinematics(m, {0.1}); }));
    // bad index
    urm::RobotModel b = make_links(2);
    b.joints.push_back(oa("j", 0, 5, JointType::Revolute, {0, 0, 0}, {0, 0, 1}));
    CHECK(throws([&] { kin::compile(b); }));
    // parent >= child
    b.joints[0].parent_link = 1; b.joints[0].child_link = 1;
    CHECK(throws([&] { kin::compile(b); }));
    // FK does not modify model
    urm::RobotModel c = tree_model();
    c.joints[0].q = 0.25;
    auto before = c.get_q();
    kin::forward_kinematics(c, {1, 2, 3});
    CHECK(c.get_q() == before);
}

static urm::RobotModel legacy_model(bool explicit_links) {
    urm::RobotModel m = make_links(4);
    for (int i = 0; i < 3; ++i) {
        Joint j; j.name = "j" + std::to_string(i);
        j.dh.a = 0.5 + 0.1 * i; j.dh.alpha = 0.2 * i; j.dh.d = 0.1;
        if (explicit_links) { j.parent_link = i; j.child_link = i + 1; }
        m.joints.push_back(j);
    }
    return m;
}

static void test_legacy_fallback() {
    auto leg = legacy_model(false), ex = legacy_model(true);
    std::vector<double> q = {0.3, -0.4, 0.9};
    auto a = kin::forward_kinematics(leg, q), b = kin::forward_kinematics(ex, q);
    CHECK((a.tcp.matrix() - b.tcp.matrix()).norm() < 1e-15);
    CHECK((kin::active_chain(leg) == std::vector<int>{0, 1, 2}));
    // model not mutated
    CHECK(leg.joints[2].parent_link == 0 && leg.joints[2].child_link == 0);
    // wrong link count in legacy
    leg.links.pop_back();
    CHECK(kin::is_legacy_serial(leg));
    CHECK(throws([&] { kin::compile(leg); }));
    // effective_links is pure: no link-count check, legacy pairs (i, i+1).
    CHECK(!throws([&] { kin::effective_links(leg); }));
    CHECK((kin::effective_links(leg)[2] == std::pair<int, int>{2, 3}));
    // active_chain uses effective_links only; tcp 2 reachable -> chain {0, 1}.
    CHECK((kin::active_chain(leg) == std::vector<int>{0, 1}));
    // mixed: not legacy -> parent >= child rule
    auto mixed = legacy_model(false);
    mixed.joints[0].parent_link = 0; mixed.joints[0].child_link = 1;
    CHECK(throws([&] { kin::compile(mixed); }));
    CHECK(!kin::is_legacy_serial(mixed));
}

// A1.4 (audit M4 + LOW compile items): compile() rejects bad input, no crash.
static void test_a14_validation() {
    // link 2 has no incoming joint (3 links, 1 joint, explicit tree links)
    auto m = make_links(3);
    m.joints.push_back(oa("a", 0, 1, JointType::Revolute, {0, 0, 0}, {0, 0, 1}));
    CHECK(!kin::is_legacy_serial(m));
    CHECK(throws([&] { kin::compile(m); }));
    // duplicate joint names
    auto d = make_links(3);
    d.joints.push_back(oa("a", 0, 1, JointType::Revolute, {0, 0, 0}, {0, 0, 1}));
    d.joints.push_back(oa("a", 1, 2, JointType::Revolute, {0, 0, 0}, {0, 0, 1}));
    CHECK(throws([&] { kin::compile(d); }));
    d.joints[1].name = "b";
    CHECK(!throws([&] { kin::compile(d); }));
    // non-finite mimic multiplier / offset, origin, axis, limit
    const double nan = std::nan("");
    auto mm = d;
    mm.joints[1].mimic = urm::Mimic{"a", nan, 0.0};
    CHECK(throws([&] { kin::compile(mm); }));
    mm.joints[1].mimic = urm::Mimic{"a", 1.0, INFINITY};
    CHECK(throws([&] { kin::compile(mm); }));
    mm.joints[1].mimic = urm::Mimic{"a", 2.0, 0.1};
    CHECK(!throws([&] { kin::compile(mm); }));
    auto o = d; o.joints[0].origin.xyz[1] = nan;
    CHECK(throws([&] { kin::compile(o); }));
    auto ax = d; ax.joints[0].axis[2] = nan;
    CHECK(throws([&] { kin::compile(ax); }));
    auto lm = d; lm.joints[1].limits.upper = INFINITY;
    CHECK(throws([&] { kin::compile(lm); }));
    auto dh = legacy_model(false); dh.joints[1].dh.a = nan;
    CHECK(throws([&] { kin::compile(dh); }));
    CHECK(!throws([&] { kin::compile(legacy_model(false)); }));   // legacy fallback still fine
}

int main() {
    test_random_chains();
    test_origin_axis();
    test_order();
    test_mimic();
    test_tree();
    test_misc();
    test_legacy_fallback();
    test_a14_validation();
    if (g_fail == 0) { std::cout << "All compiled-model tests passed.\n"; return 0; }
    std::cerr << g_fail << " check(s) FAILED\n";
    return 1;
}
