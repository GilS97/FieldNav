#include <gtest/gtest.h>

#include <cmath>
#include <fieldnav/sim/diff_drive_sim.hpp>
#include <stdexcept>
#include <string>

namespace sim = fieldnav::sim;

namespace {

constexpr double kPi = 3.14159265358979323846;
const std::string kRobot = std::string(FIELDNAV_SCENES_DIR) + "/robot_diffdrive.xml";

// Fait tourner la sim pendant `seconds` avec une consigne constante.
void run(sim::DiffDriveSim& s, sim::WheelSpeeds cmd, double seconds) {
    const auto steps = static_cast<int>(std::lround(seconds / s.timestep()));
    s.setWheelSpeeds(cmd);
    for (int i = 0; i < steps; ++i) {
        s.step();
    }
}

} // namespace

// Robot : LAFVIN 4WD (scenes/robot_diffdrive.xml). Valeurs mesurées sur ce modèle le
// 2 nov. 2026 : 0,84 kg ; ligne droite à 10 rad/s → 0,321 m/s (r·ω à 0,2 % près) ;
// virage 5 / 10 rad/s → ω réel ≈ 0,53 rad/s contre 0,77 prévu par la voie géométrique.

// ─── Chargement et géométrie ───

TEST(DiffDriveSim, LoadsRobotGeometry) {
    const sim::DiffDriveSim s(kRobot);
    EXPECT_NEAR(s.wheelRadius(), 0.0325, 1e-12); // roue TT de 65 mm
    EXPECT_NEAR(s.trackWidth(), 0.150, 1e-12);
    EXPECT_NEAR(s.timestep(), 0.002, 1e-12);
    EXPECT_DOUBLE_EQ(s.time(), 0.0);
}

TEST(DiffDriveSim, MissingModelThrows) {
    EXPECT_THROW(sim::DiffDriveSim("n_existe_pas.xml"), std::runtime_error);
}

// ─── Pose vraie et vitesses de roues ───

TEST(DiffDriveSim, StartsAtOriginFacingForward) {
    const sim::DiffDriveSim s(kRobot);
    const auto p = s.truePose();
    EXPECT_NEAR(p.x, 0.0, 1e-9);
    EXPECT_NEAR(p.y, 0.0, 1e-9);
    EXPECT_NEAR(p.theta, 0.0, 1e-9);
}

TEST(DiffDriveSim, WheelsReachCommandedSpeed) {
    sim::DiffDriveSim s(kRobot);
    run(s, {10.0, 10.0}, 2.0);
    const auto w = s.wheelSpeeds();
    EXPECT_NEAR(w.left, 10.0, 0.2); // 2 %
    EXPECT_NEAR(w.right, 10.0, 0.2);
}

TEST(DiffDriveSim, StraightLineAdvancesAtWheelSpeedTimesRadius) {
    sim::DiffDriveSim s(kRobot);
    run(s, {10.0, 10.0}, 2.0); // mise en régime
    const double x0 = s.truePose().x;
    const double v_wheels = s.wheelRadius() * (s.wheelSpeeds().left + s.wheelSpeeds().right) / 2.0;
    run(s, {10.0, 10.0}, 1.0);
    const auto p = s.truePose();
    EXPECT_GT(v_wheels, 0.25); // échoue si wheelSpeeds() renvoie 0
    // En ligne droite les 4 roues motrices ne patinent presque pas (mesuré : 0,2 %).
    EXPECT_NEAR(p.x - x0, v_wheels * 1.0, 0.02 * v_wheels); // 2 %
    EXPECT_NEAR(p.y, 0.0, 1e-3);
    EXPECT_NEAR(p.theta, 0.0, 1e-3);
}

TEST(DiffDriveSim, TurnsLeftWhenRightWheelIsFaster) {
    sim::DiffDriveSim s(kRobot);
    run(s, {5.0, 10.0}, 2.0);
    const auto p = s.truePose();
    EXPECT_GT(p.theta, 0.5); // cap positif = vers la gauche
    EXPECT_GT(p.y, 0.0);
}

TEST(DiffDriveSim, SkidSteerTurnsLessThanGeometricModel) {
    // Un châssis 4 roues tourne en faisant glisser ses roues : il tourne moins vite que
    // r·(ωd − ωg)/voie. D'où l'entraxe effectif L_eff > voie, à étalonner (ici ≈ 0,22 m).
    sim::DiffDriveSim s(kRobot);
    run(s, {5.0, 10.0}, 2.0); // mise en régime
    const double theta0 = s.truePose().theta;
    const auto w = s.wheelSpeeds();
    run(s, {5.0, 10.0}, 0.5);
    const double omega_measured = (s.truePose().theta - theta0) / 0.5;
    const double omega_geometric = s.wheelRadius() * (w.right - w.left) / s.trackWidth();
    EXPECT_GT(omega_measured, 0.0);
    EXPECT_LT(omega_measured, 0.9 * omega_geometric);
    const double l_eff = s.wheelRadius() * (w.right - w.left) / omega_measured;
    EXPECT_GT(l_eff, s.trackWidth());
    EXPECT_LT(l_eff, 2.0 * s.trackWidth());
}

TEST(DiffDriveSim, HeadingStaysInRangeWhileSpinning) {
    sim::DiffDriveSim s(kRobot);
    double previous = 0.0;
    bool wrapped = false;
    s.setWheelSpeeds({-5.0, 5.0});
    for (int i = 0; i < 5000; ++i) { // 10 s, plusieurs tours sur place
        s.step();
        const double theta = s.truePose().theta;
        EXPECT_GE(theta, -kPi);
        EXPECT_LE(theta, kPi);
        if (previous > 3.0 && theta < -3.0) {
            wrapped = true; // passage de +π à -π : le cap a bien fait un tour
        }
        previous = theta;
    }
    EXPECT_TRUE(wrapped);
}

TEST(DiffDriveSim, SameCommandsGiveIdenticalRuns) {
    sim::DiffDriveSim a(kRobot);
    sim::DiffDriveSim b(kRobot);
    run(a, {7.0, 9.0}, 1.5);
    run(b, {7.0, 9.0}, 1.5);
    EXPECT_DOUBLE_EQ(a.truePose().x, b.truePose().x); // reproductible octet pour octet
    EXPECT_DOUBLE_EQ(a.truePose().y, b.truePose().y);
    EXPECT_DOUBLE_EQ(a.truePose().theta, b.truePose().theta);
    EXPECT_GT(a.truePose().x, 0.1);
}

TEST(DiffDriveSim, ResetReturnsToOrigin) {
    sim::DiffDriveSim s(kRobot);
    run(s, {10.0, 8.0}, 1.0);
    s.reset();
    EXPECT_DOUBLE_EQ(s.time(), 0.0);
    EXPECT_NEAR(s.truePose().x, 0.0, 1e-9);
    EXPECT_NEAR(s.truePose().theta, 0.0, 1e-9);
    EXPECT_NEAR(s.wheelSpeeds().left, 0.0, 1e-12);
}

// ─── Scène de rang (générée par scripts/gen_row_scene.py) ───

TEST(DiffDriveSim, LoadsStraightRowSceneWithRobotAtStart) {
    // La scène inclut le robot : mêmes noms, même géométrie, départ à l'origine au milieu du rang.
    const sim::DiffDriveSim s(std::string(FIELDNAV_SCENES_DIR) + "/row_straight.xml");
    EXPECT_NEAR(s.wheelRadius(), 0.0325, 1e-12);
    const auto p = s.truePose();
    EXPECT_NEAR(p.x, 0.0, 1e-9);
    EXPECT_NEAR(p.y, 0.0, 1e-9);
    EXPECT_NEAR(p.theta, 0.0, 1e-9);
}

TEST(DiffDriveSim, DrivesStraightDownTheRow) {
    // 10 s tout droit dans le rang : les plants n'ont pas de collision, le robot avance librement.
    sim::DiffDriveSim s(std::string(FIELDNAV_SCENES_DIR) + "/row_straight.xml");
    run(s, {10.0, 10.0}, 10.0);
    EXPECT_NEAR(s.truePose().x, 3.2, 0.2);
    EXPECT_NEAR(s.truePose().y, 0.0, 0.01);
}
