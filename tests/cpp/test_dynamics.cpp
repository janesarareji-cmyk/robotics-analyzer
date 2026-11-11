/**
 * test_dynamics.cpp
 * R4.6: inverse dynamics (RNEA) against closed-form cases A, B, C.
 */
#include <cmath>
#include <iostream>
#include <random>
#include <stdexcept>

#include "dyn/dynamics.hpp"
#include "kin/compiled_model.hpp"
#include "kin/fk.hpp"
#include "urm/robot_model.hpp"

using urm::Joint; using urm::JointType; using urm::JointKinematics;

static int g_fail = 0;
#define CHECK(c) do { if (!(c)) { std::cerr << "FAIL line " << __LINE__ << ": " #c "\n"; ++g_fail; } } while (0)

static double g_worst = 0.0;
static void near(double got, double want, double tol, int line) {
    const double err = std::abs(got - want) / std::max(1.0, std::abs(want));
    if (err > g_worst) g_worst = err;
    if (!(err <= tol)) {
        std::cerr << "FAIL line " << line << ": got " << got << " want " << want << " err " << err << "\n";
        ++g_fail;
    }
}
#define NEAR(a, b, tol) near((a), (b), (tol), __LINE__)

static urm::Link link_with(const std::string& name, double m, std::array<double, 3> c,
                           std::array<double, 6> I) {
    urm::Link l; l.name = name;
    l.inertia.mass = m; l.inertia.com = c; l.inertia.inertia_tensor = I;
    return l;
}

static Joint oa(const std::string& name, int p, int c, JointType t,
                std::array<double, 3> xyz, std::array<double, 3> axis,
                std::array<double, 3> rpy = {0, 0, 0}) {
    Joint j; j.name = name; j.parent_link = p; j.child_link = c; j.type = t;
    j.kinematics = JointKinematics::OriginAxis;
    j.origin.xyz = xyz; j.origin.rpy = rpy; j.axis = axis;
    return j;
}

static std::array<double, 3> rpy_of(const Eigen::Matrix3d& R) {
    // R = Rz(y) Ry(p) Rx(r)
    return {std::atan2(R(2, 1), R(2, 2)), -std::asin(R(2, 0)), std::atan2(R(1, 0), R(0, 0))};
}

static std::vector<double> rnea(const urm::RobotModel& m, const std::vector<double>& q,
                                const std::vector<double>& qd, const std::vector<double>& qdd,
                                const Eigen::Vector3d& g) {
    const kin::CompiledModel cm = kin::compile(m);
    const dyn::DynModel dm = dyn::build_dyn_model(m, cm);
    return dyn::inverse_dynamics(dm, q, qd, qdd, g);
}

static dyn::DynModel dmodel(const urm::RobotModel& m) {
    return dyn::build_dyn_model(m, kin::compile(m));
}

// ---------------------------------------------------------------- case A
static void case_A() {
    const double l1 = 0.7, m1 = 1.3, m2 = 0.9, r1 = 0.31, r2 = 0.22, I1 = 0.05, I2 = 0.03, g0 = 9.81;
    urm::RobotModel m;
    m.links.push_back(urm::Link{"base"});
    m.links.push_back(link_with("l1", m1, {r1, 0, 0}, {0, 0, 0, 0, 0, I1}));
    m.links.push_back(link_with("l2", m2, {r2, 0, 0}, {0, 0, 0, 0, 0, I2}));
    m.joints.push_back(oa("j1", 0, 1, JointType::Revolute, {0, 0, 0}, {0, 0, 1}));
    m.joints.push_back(oa("j2", 1, 2, JointType::Revolute, {l1, 0, 0}, {0, 0, 1}));
    std::mt19937 rng(1);
    std::uniform_real_distribution<double> U(-2.0, 2.0);
    for (int t = 0; t < 50; ++t) {
        const double q1 = U(rng), q2 = U(rng), d1 = U(rng), d2 = U(rng), a1 = U(rng), a2 = U(rng);
        const double c1 = std::cos(q1), c2 = std::cos(q2), s2 = std::sin(q2), c12 = std::cos(q1 + q2);
        const double M11 = I1 + I2 + m1 * r1 * r1 + m2 * (l1 * l1 + r2 * r2 + 2 * l1 * r2 * c2);
        const double M12 = I2 + m2 * (r2 * r2 + l1 * r2 * c2);
        const double M22 = I2 + m2 * r2 * r2;
        const double C1 = -m2 * l1 * r2 * s2 * (2 * d1 * d2 + d2 * d2);
        const double C2 = m2 * l1 * r2 * s2 * d1 * d1;
        const double G1 = g0 * ((m1 * r1 + m2 * l1) * c1 + m2 * r2 * c12);
        const double G2 = g0 * m2 * r2 * c12;
        const Eigen::Vector3d g(0, -g0, 0);
        const auto tau = rnea(m, {q1, q2}, {d1, d2}, {a1, a2}, g);
        NEAR(tau[0], M11 * a1 + M12 * a2 + C1 + G1, 1e-10);
        NEAR(tau[1], M12 * a1 + M22 * a2 + C2 + G2, 1e-10);
        const auto dm = dmodel(m);
        const Eigen::MatrixXd M = dyn::mass_matrix(dm, {q1, q2});
        NEAR(M(0, 0), M11, 1e-12); NEAR(M(0, 1), M12, 1e-12);
        NEAR(M(1, 0), M12, 1e-12); NEAR(M(1, 1), M22, 1e-12);
        // forward dynamics inverts RNEA, with friction too
        dyn::Friction fr; fr.viscous = {0.3, 0.2}; fr.coulomb = {0.1, 0.05}; fr.eps = 1e-2;
        const auto tf = dyn::inverse_dynamics(dm, {q1, q2}, {d1, d2}, {a1, a2}, g, &fr);
        NEAR(tf[0], tau[0] + 0.3 * d1 + 0.1 * std::tanh(d1 / 1e-2), 1e-12);
        NEAR(tf[1], tau[1] + 0.2 * d2 + 0.05 * std::tanh(d2 / 1e-2), 1e-12);
        const auto qdd = dyn::forward_dynamics(dm, {q1, q2}, {d1, d2}, tf, g, &fr);
        NEAR(qdd[0], a1, 1e-10); NEAR(qdd[1], a2, 1e-10);
    }
    // no trig: q = 0, qd = 0
    const auto tau = rnea(m, {0, 0}, {0, 0}, {1.0, -0.5}, Eigen::Vector3d(0, -g0, 0));
    const double M11 = I1 + I2 + m1 * r1 * r1 + m2 * (l1 * l1 + r2 * r2 + 2 * l1 * r2);
    const double M12 = I2 + m2 * (r2 * r2 + l1 * r2), M22 = I2 + m2 * r2 * r2;
    NEAR(tau[0], M11 * 1.0 + M12 * -0.5 + g0 * ((m1 * r1 + m2 * l1) + m2 * r2), 1e-12);
    NEAR(tau[1], M12 * 1.0 + M22 * -0.5 + g0 * m2 * r2, 1e-12);
}

// ---------------------------------------------------------------- case B
static void case_B() {
    const double m1 = 2.0, m2 = 0.8, r = 0.4, I2 = 0.04, g0 = 9.81;
    urm::RobotModel m;
    m.links.push_back(urm::Link{"base"});
    m.links.push_back(link_with("cart", m1, {0, 0, 0}, {0, 0, 0, 0, 0, 0}));
    m.links.push_back(link_with("pend", m2, {r, 0, 0}, {0, 0, 0, 0, 0, I2}));
    m.joints.push_back(oa("slide", 0, 1, JointType::Prismatic, {0, 0, 0}, {1, 0, 0}));
    m.joints.push_back(oa("pivot", 1, 2, JointType::Revolute, {0, 0, 0}, {0, 0, 1}));
    std::mt19937 rng(2);
    std::uniform_real_distribution<double> U(-2.0, 2.0);
    for (int t = 0; t < 50; ++t) {
        const double x = U(rng), th = U(rng), dx = U(rng), dth = U(rng), ax = U(rng), ath = U(rng);
        const double st = std::sin(th), ct = std::cos(th);
        const double M11 = m1 + m2, M12 = -m2 * r * st, M22 = I2 + m2 * r * r;
        const double C1 = -m2 * r * ct * dth * dth;
        const double G2 = m2 * g0 * r * ct;
        const auto tau = rnea(m, {x, th}, {dx, dth}, {ax, ath}, Eigen::Vector3d(0, -g0, 0));
        NEAR(tau[0], M11 * ax + M12 * ath + C1, 1e-10);
        NEAR(tau[1], M12 * ax + M22 * ath + G2, 1e-10);
        const Eigen::MatrixXd M = dyn::mass_matrix(dmodel(m), {x, th});
        NEAR(M(0, 0), M11, 1e-12); NEAR(M(0, 1), M12, 1e-12);
        NEAR(M(1, 0), M12, 1e-12); NEAR(M(1, 1), M22, 1e-12);
    }
}

// ---------------------------------------------------------------- case C
// Revolute z with DH post = Tx(l) Rx(90 deg): joint axis is the link y axis.
static urm::RobotModel case_C_model(double l, double mass, std::array<double, 3> c,
                                    std::array<double, 6> Ic, bool variant, double w,
                                    Eigen::Matrix3d W) {
    urm::RobotModel m;
    m.convention = urm::DHConvention::Standard;
    m.links.push_back(urm::Link{"base"});
    Joint j;
    j.name = "jc"; j.kinematics = JointKinematics::DH; j.type = JointType::Revolute;
    j.dh = {l, M_PI / 2, 0.0, 0.0};
    if (!variant) {
        m.links.push_back(link_with("l1", mass, c, Ic));
        j.parent_link = 0; j.child_link = 1;
        m.joints.push_back(j);
    } else {
        // base = W * Ry(-37deg); fixed joint pre = Ry(37deg): link 1 frame = W.
        const double a = 37.0 * M_PI / 180.0;
        const Eigen::Matrix3d Ry = Eigen::AngleAxisd(a, Eigen::Vector3d::UnitY()).toRotationMatrix();
        const Eigen::Matrix3d Rb = W * Ry.transpose();
        m.base.xyz = {0.4, -0.2, 0.9};
        m.base.rpy = rpy_of(Rb);
        m.links.push_back(urm::Link{"mid"});
        m.links.push_back(link_with("l2", mass, c, Ic));
        m.joints.push_back(oa("fix", 0, 1, JointType::Fixed, {0.1, 0.2, -0.3}, {0, 0, 1}, {0, a, 0}));
        j.parent_link = 1; j.child_link = 2;
        // fixed joint origin translation would move link 1; take it out by symmetry:
        // frame 1 = base * T(fix) so the offset is part of the world pose only.
        m.joints.push_back(j);
        (void)w;
    }
    return m;
}

static void case_C() {
    const double l = 0.6, mass = 1.7, d = 0.25, g0 = 9.81;
    const double Ixx = 0.11, Iyy = 0.07, Izz = 0.19;
    std::mt19937 rng(3);
    std::uniform_real_distribution<double> U(-2.0, 2.0);
    for (int variant = 0; variant < 2; ++variant) {
        Eigen::Matrix3d W = Eigen::Matrix3d::Identity();
        if (variant) {
            W = (Eigen::AngleAxisd(0.8, Eigen::Vector3d::UnitZ()) *
                 Eigen::AngleAxisd(-0.5, Eigen::Vector3d::UnitY()) *
                 Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitX())).toRotationMatrix();
        }
        const urm::RobotModel m = case_C_model(l, mass, {0, 0, d}, {Ixx, 0, 0, Iyy, 0, Izz}, variant != 0, 0.8, W);
        const Eigen::Vector3d g = W * Eigen::Vector3d(0, -g0, 0);
        for (int t = 0; t < 50; ++t) {
            const double q = U(rng), qd = U(rng), qdd = U(rng);
            const double M = Iyy + mass * (l * l + d * d);
            const double G = mass * g0 * (l * std::cos(q) + d * std::sin(q));
            const auto tau = rnea(m, {q}, {qd}, {qdd}, g);
            NEAR(tau[0], M * qdd + G, 1e-10);   // C = 0 for one DOF
            NEAR(dyn::mass_matrix(dmodel(m), {q})(0, 0), M, 1e-10);
        }
    }
}

// Frames hook equals FK; wrong sizes throw; zero gravity rest gives zero.
static void misc() {
    const double l1 = 0.7;
    urm::RobotModel m;
    m.links.push_back(urm::Link{"base"});
    m.links.push_back(link_with("l1", 1.0, {0.1, 0, 0}, {0.01, 0, 0, 0.02, 0, 0.03}));
    m.links.push_back(link_with("l2", 1.0, {0.1, 0.05, 0}, {0.01, 0, 0, 0.02, 0, 0.03}));
    m.joints.push_back(oa("j1", 0, 1, JointType::Revolute, {0, 0, 0}, {0, 0, 1}));
    m.joints.push_back(oa("j2", 1, 2, JointType::Revolute, {l1, 0, 0.1}, {0, 1, 0}, {0.3, 0.2, 0.1}));
    const kin::CompiledModel cm = kin::compile(m);
    const dyn::DynModel dm = dyn::build_dyn_model(m, cm);
    const std::vector<double> q = {0.4, -0.9};
    const auto fr = dyn::dyn_frames(dm, q);
    const auto fk = kin::forward_kinematics(cm, q);
    for (size_t i = 0; i < fr.size(); ++i) CHECK((fr[i].matrix() - fk.frames[i].matrix()).norm() < 1e-12);
    bool thrown = false;
    try { dyn::inverse_dynamics(dm, {0.0}, {0, 0}, {0, 0}, Eigen::Vector3d::Zero()); }
    catch (const std::invalid_argument&) { thrown = true; }
    CHECK(thrown);
    const auto t0 = dyn::inverse_dynamics(dm, q, {0, 0}, {0, 0}, Eigen::Vector3d::Zero());
    CHECK(std::abs(t0[0]) < 1e-15 && std::abs(t0[1]) < 1e-15);
}

// Massless leaf: M singular -> runtime_error; armature fixes it. Wrong sizes throw.
static void singular_and_sizes() {
    urm::RobotModel m;
    m.links.push_back(urm::Link{"base"});
    m.links.push_back(urm::Link{"massless"});
    m.joints.push_back(oa("j1", 0, 1, JointType::Revolute, {0, 0, 0}, {0, 0, 1}));
    const dyn::DynModel dm = dmodel(m);
    bool thrown = false;
    try { dyn::forward_dynamics(dm, {0.0}, {0.0}, {0.0}, Eigen::Vector3d::Zero()); }
    catch (const std::runtime_error&) { thrown = true; }
    CHECK(thrown);
    const std::vector<double> arm = {0.5};
    const auto qdd = dyn::forward_dynamics(dm, {0.0}, {0.0}, {1.0}, Eigen::Vector3d::Zero(), nullptr, &arm);
    NEAR(qdd[0], 2.0, 1e-14);
    thrown = false;
    try { dyn::forward_dynamics(dm, {0.0}, {0.0}, {}, Eigen::Vector3d::Zero()); }
    catch (const std::invalid_argument&) { thrown = true; }
    CHECK(thrown);
    thrown = false;
    try { dyn::mass_matrix(dm, {}); }
    catch (const std::invalid_argument&) { thrown = true; }
    CHECK(thrown);
}

int main() {
    singular_and_sizes();
    case_A();
    case_B();
    case_C();
    misc();
    std::cout << "worst relative error: " << g_worst << "\n";
    if (g_fail) { std::cerr << g_fail << " failure(s)\n"; return 1; }
    std::cout << "test_dynamics OK\n";
    return 0;
}
