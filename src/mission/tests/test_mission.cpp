// Tests d'intégration de la mission complète (rang droit de 50 m, scène générée).
#include <gtest/gtest.h>

#include <cstdio>
#include <fieldnav/mission/mission.hpp>
#include <fstream>
#include <stdexcept>
#include <string>

namespace fm = fieldnav::mission;

namespace {

fm::MissionConfig baseConfig(fm::PoseSource source) {
    fm::MissionConfig c;
    c.scene_file = std::string(FIELDNAV_SCENES_DIR) + "/row_straight.xml";
    c.reference_file = std::string(FIELDNAV_DATA_DIR) + "/reference_straight.csv";
    c.pose_source = source;
    c.seed = 42;
    return c;
}

int countLines(const std::string& file) {
    std::ifstream in(file);
    int n = 0;
    std::string line;
    while (std::getline(in, line)) {
        ++n;
    }
    return n;
}

} // namespace

TEST(Mission, ParsesPoseSource) {
    EXPECT_EQ(fm::parsePoseSource("truth"), fm::PoseSource::Truth);
    EXPECT_EQ(fm::parsePoseSource("odom"), fm::PoseSource::Odometry);
    EXPECT_EQ(fm::parsePoseSource("fusion"), fm::PoseSource::Fusion);
    EXPECT_FALSE(fm::parsePoseSource("gps").has_value());
}

TEST(Mission, TruthFollowsTheRowPerfectly) {
    const auto r = fm::runMission(baseConfig(fm::PoseSource::Truth));
    EXPECT_TRUE(r.reached_end);
    EXPECT_FALSE(r.off_row);
    EXPECT_EQ(r.exitCode(), 0);
    EXPECT_GT(r.distance, 49.9);
    EXPECT_LT(r.rms_lateral, 0.01);
}

TEST(Mission, FusionCompletesTheRowWithGnssOutage) {
    // Conditions de la métrique (CLAUDE.md) : GPS σ = 1 m, 20 % de pertes, trou de 10 s au
    // milieu du rang (≈ 85 s à 0,3 m/s). Mesuré le 27 nov. 2026 (seed 42) : RMS 0,144 m, max 0,309
    // m.
    auto c = baseConfig(fm::PoseSource::Fusion);
    c.gps.outage_start = 80.0;
    c.gps.outage_end = 90.0;
    const auto r = fm::runMission(c);
    std::printf("fusion, seed 42 : RMS latérale %.3f m, max %.3f m, %.1f m en %.0f s\n",
                r.rms_lateral, r.max_lateral, r.distance, r.duration);
    EXPECT_TRUE(r.reached_end);
    EXPECT_EQ(r.exitCode(), 0);
    EXPECT_LT(r.rms_lateral, 0.3);
}

TEST(Mission, OdometryAloneDriftsMoreThanFusion) {
    auto odo = baseConfig(fm::PoseSource::Odometry);
    auto fus = baseConfig(fm::PoseSource::Fusion);
    odo.max_duration = fus.max_duration = 120.0;
    EXPECT_GT(fm::runMission(odo).rms_lateral, fm::runMission(fus).rms_lateral);
}

TEST(Mission, SameSeedSameResult) {
    auto c = baseConfig(fm::PoseSource::Fusion);
    c.max_duration = 20.0;
    const auto a = fm::runMission(c);
    const auto b = fm::runMission(c);
    EXPECT_DOUBLE_EQ(a.rms_lateral, b.rms_lateral);
    EXPECT_DOUBLE_EQ(a.distance, b.distance);
}

TEST(Mission, OffRowIsDetected) {
    auto c = baseConfig(fm::PoseSource::Fusion);
    c.off_row_limit = 0.001; // 1 mm : la fusion s'en écarte en quelques secondes
    const auto r = fm::runMission(c);
    EXPECT_TRUE(r.off_row);
    EXPECT_EQ(r.exitCode(), 1);
}

TEST(Mission, WritesTelemetryAtTenHertz) {
    auto c = baseConfig(fm::PoseSource::Fusion);
    c.max_duration = 5.0;
    c.telemetry_file = ::testing::TempDir() + "mission.csv";
    const auto r = fm::runMission(c);
    EXPECT_EQ(r.telemetry_samples, 50U);
    EXPECT_EQ(countLines(c.telemetry_file), 51); // en-tête + 50 lignes
}

TEST(Mission, MissingSceneThrows) {
    auto c = baseConfig(fm::PoseSource::Truth);
    c.scene_file = "n_existe_pas.xml";
    EXPECT_THROW((void)fm::runMission(c), std::runtime_error);
}
