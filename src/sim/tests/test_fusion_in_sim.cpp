// Test d'intégration du jalon Capteurs : sim → roues bruitées → odométrie, sim → GPS bruité,
// fusion des deux, et RMS odo / gps / fusion imprimés en fin de run.
// Critère de fini (cap2027-details-taches.json) : RMS_fusion < RMS_odo et RMS_fusion < RMS_gps.
#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <fieldnav/estimation/complementary.hpp>
#include <fieldnav/odometry/kinematics.hpp>
#include <fieldnav/odometry/odometry.hpp>
#include <fieldnav/sensor_utils/stats.hpp>
#include <fieldnav/sensors/gps_sim.hpp>
#include <fieldnav/sensors/wheel_noise.hpp>
#include <fieldnav/sim/diff_drive_sim.hpp>
#include <optional>
#include <string>
#include <vector>

namespace {

const std::string kRobot = std::string(FIELDNAV_SCENES_DIR) + "/robot_diffdrive.xml";
constexpr double kTrackEff = 0.22; // entraxe effectif du châssis 4 roues (mesuré en sim)

struct RmsResult {
    double odo{0.0};
    double gps{0.0};
    double fusion{0.0};
};

// Run de référence : `seconds` de roulage avec virages alternés (8 s tout droit, 8 s à gauche,
// 8 s tout droit, 8 s à droite...), GPS par défaut (σ 1 m, 20 % de pertes, 5 Hz) avec un trou
// GNSS de 10 s au milieu. Les virages font dériver l'odométrie (entraxe effectif imparfait).
RmsResult runReference(std::uint32_t seed, double seconds) {
    namespace odo = fieldnav::odometry;
    namespace sen = fieldnav::sensors;
    fieldnav::sim::DiffDriveSim sim(kRobot);
    sen::WheelNoise noise_left(seed), noise_right(seed + 1);
    sen::GpsParams gps_params;
    gps_params.outage_start = seconds / 2.0 - 5.0;
    gps_params.outage_end = seconds / 2.0 + 5.0;
    sen::GpsSim gps(seed + 2, gps_params);
    odo::DiffDriveOdometry odom;
    fieldnav::estimation::ComplementaryFuser fusion;
    std::optional<sen::GpsFix> last_fix; // « GPS seul » = dernière mesure reçue, tenue

    std::vector<double> err_odo, err_gps, err_fusion;
    const double dt = sim.timestep();
    const auto steps = static_cast<int>(std::lround(seconds / dt));
    for (int i = 1; i <= steps; ++i) {
        const double t = i * dt;
        const int phase = static_cast<int>(t / 8.0) % 4;
        const double left = phase == 1 ? 9.0 : (phase == 3 ? 11.0 : 10.0);
        const double right = phase == 1 ? 11.0 : (phase == 3 ? 9.0 : 10.0);
        sim.setWheelSpeeds({left, right});
        sim.step();

        const auto wheels = sim.wheelSpeeds();
        const auto twist =
            odo::wheelsToTwist({noise_left.apply(wheels.left), noise_right.apply(wheels.right)},
                               sim.wheelRadius(), kTrackEff);
        odom.update(twist.v, twist.w, dt);
        fusion.predict(twist.v, twist.w, dt);

        const auto truth = sim.truePose();
        if (const auto fix = gps.update(t, truth.x, truth.y)) {
            fusion.correct(*fix);
            last_fix = fix;
        }

        const auto dist = [&](double x, double y) { return std::hypot(x - truth.x, y - truth.y); };
        err_odo.push_back(dist(odom.pose().x, odom.pose().y));
        err_fusion.push_back(dist(fusion.estimate().x, fusion.estimate().y));
        if (last_fix) {
            err_gps.push_back(dist(last_fix->x, last_fix->y));
        }
    }
    namespace su = fieldnav::sensor_utils;
    return {su::rms(err_odo), su::rms(err_gps), su::rms(err_fusion)};
}

} // namespace

TEST(FusionInSim, FusionBeatsOdometryAndGpsAlone) {
    const auto r = runReference(42, 120.0);
    std::printf("RMS position (120 s, virages, trou GNSS 10 s, seed 42) :\n"
                "  odométrie seule : %.3f m\n  GPS seul        : %.3f m\n  fusion          : "
                "%.3f m\n",
                r.odo, r.gps, r.fusion);
    ASSERT_TRUE(std::isfinite(r.fusion));
    EXPECT_LT(r.fusion, r.odo); // critère de fini de la tâche
    EXPECT_LT(r.fusion, r.gps);
    EXPECT_LT(r.fusion, 0.5); // mesuré le 9 nov. 2026 : 0,28 m
}

TEST(FusionInSim, SameSeedSameRms) {
    const auto a = runReference(42, 20.0);
    const auto b = runReference(42, 20.0);
    EXPECT_DOUBLE_EQ(a.fusion, b.fusion);
    EXPECT_DOUBLE_EQ(a.odo, b.odo);
}
