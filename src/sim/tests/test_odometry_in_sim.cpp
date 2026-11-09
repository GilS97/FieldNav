// Test d'intégration : la chaîne complète sim → bruit des roues → (v, ω) → odométrie.
// C'est la boucle qui ira dans apps/fieldnav_run (jalon Contrôle) ; en attendant, elle vit ici.
#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <fieldnav/odometry/kinematics.hpp>
#include <fieldnav/odometry/odometry.hpp>
#include <fieldnav/sensors/wheel_noise.hpp>
#include <fieldnav/sim/diff_drive_sim.hpp>
#include <string>

namespace {

const std::string kRobot = std::string(FIELDNAV_SCENES_DIR) + "/robot_diffdrive.xml";
constexpr double kTrackEff = 0.22; // entraxe effectif du châssis 4 roues (mesuré en sim)

struct RunResult {
    fieldnav::odometry::Pose2D truth;
    fieldnav::odometry::Pose2D estimate;
    double error_at_10s{0.0}; // écart de position estimée / vraie à t = 10 s (m)
    double error_at_end{0.0};
    bool finite{true}; // aucune NaN ni infini pendant tout le run
};

double distance(const fieldnav::odometry::Pose2D& a, const fieldnav::odometry::Pose2D& b) {
    return std::hypot(a.x - b.x, a.y - b.y);
}

// Roule `seconds` en ligne droite (10 rad/s sur les deux côtés ≈ 0,32 m/s) et suit l'odométrie.
RunResult runStraight(std::uint32_t seed, double seconds) {
    namespace odo = fieldnav::odometry;
    fieldnav::sim::DiffDriveSim sim(kRobot);
    // Un générateur par roue, seeds dérivées de la seed du run : les deux bruits sont
    // indépendants et le run reste reproductible.
    fieldnav::sensors::WheelNoise noise_left(seed);
    fieldnav::sensors::WheelNoise noise_right(seed + 1);
    odo::DiffDriveOdometry odom;

    RunResult r;
    sim.setWheelSpeeds({10.0, 10.0});
    const double dt = sim.timestep();
    const auto steps = static_cast<int>(std::lround(seconds / dt));
    const auto step_10s = static_cast<int>(std::lround(10.0 / dt));
    for (int i = 1; i <= steps; ++i) {
        sim.step();
        const auto measured = sim.wheelSpeeds();
        const odo::WheelRates noisy{noise_left.apply(measured.left),
                                    noise_right.apply(measured.right)};
        const auto twist = odo::wheelsToTwist(noisy, sim.wheelRadius(), kTrackEff);
        odom.update(twist.v, twist.w, dt);

        const auto& est = odom.pose();
        if (!std::isfinite(est.x) || !std::isfinite(est.y) || !std::isfinite(est.theta)) {
            r.finite = false;
        }
        if (i == step_10s) {
            r.error_at_10s = distance(est, sim.truePose());
        }
    }
    r.truth = sim.truePose();
    r.estimate = odom.pose();
    r.error_at_end = distance(r.estimate, r.truth);
    return r;
}

} // namespace

TEST(OdometryInSim, SixtySecondsStraightDriftsSlowly) {
    const auto r = runStraight(42, 60.0);
    ASSERT_TRUE(r.finite);
    // Le robot vrai a bien roulé : ≈ 0,32 m/s × 60 s ≈ 19 m, en restant dans l'axe.
    EXPECT_NEAR(r.truth.x, 19.3, 0.5);
    EXPECT_NEAR(r.truth.y, 0.0, 0.05);
    // L'odométrie suit le robot (pas de facteur 2, pas de signe faux)...
    EXPECT_NEAR(r.estimate.x, r.truth.x, 0.5);
    // ... mais dérive : l'écart grandit avec le temps, sans exploser.
    EXPECT_GT(r.error_at_end, 0.01);
    EXPECT_GT(r.error_at_end, r.error_at_10s);
    EXPECT_LT(r.error_at_end, 1.0);
    std::printf("dérive odométrie : %.3f m à 10 s, %.3f m à 60 s (x vrai = %.2f m)\n",
                r.error_at_10s, r.error_at_end, r.truth.x);
}

TEST(OdometryInSim, SameSeedGivesIdenticalRun) {
    const auto a = runStraight(42, 5.0);
    const auto b = runStraight(42, 5.0);
    EXPECT_DOUBLE_EQ(a.estimate.x, b.estimate.x); // même seed = même résultat, octet pour octet
    EXPECT_DOUBLE_EQ(a.estimate.y, b.estimate.y);
    EXPECT_DOUBLE_EQ(a.estimate.theta, b.estimate.theta);
}

TEST(OdometryInSim, DifferentSeedsGiveDifferentDrift) {
    const auto a = runStraight(1, 5.0);
    const auto b = runStraight(2, 5.0);
    EXPECT_NE(a.estimate.y, b.estimate.y);
}
