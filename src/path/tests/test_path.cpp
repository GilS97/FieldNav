#include <gtest/gtest.h>

#include <cmath>
#include <fieldnav/path/path.hpp>
#include <stdexcept>
#include <string>
#include <vector>

namespace fp = fieldnav::path;
using fieldnav::odometry::Pose2D;

namespace {

constexpr double kPi = 3.14159265358979323846;

// Ligne droite le long de x, de 0 à `length`, un point tous les 10 cm (comme le rang).
fp::Path straight(double length = 10.0) {
    std::vector<fp::RefPoint> pts;
    const auto n = static_cast<int>(std::lround(length / 0.1));
    for (int i = 0; i <= n; ++i) {
        const double s = i * 0.1;
        pts.push_back({s, s, 0.0});
    }
    return fp::Path(pts);
}

// Arc de cercle de rayon R tournant à gauche, centre (0, R), parcouru sur un quart de tour.
fp::Path arc(double radius) {
    std::vector<fp::RefPoint> pts;
    const auto n = static_cast<int>(std::lround(radius * kPi / 2.0 / 0.1));
    for (int i = 0; i <= n; ++i) {
        const double s = i * 0.1;
        pts.push_back({s, radius * std::sin(s / radius), radius * (1.0 - std::cos(s / radius))});
    }
    return fp::Path(pts);
}

} // namespace

// ─── Construction et recherche du point le plus proche (déjà écrits) ───

TEST(Path, RejectsTooFewPointsOrNonIncreasingS) {
    EXPECT_THROW(fp::Path({{0.0, 0.0, 0.0}}), std::invalid_argument);
    EXPECT_THROW(fp::Path({{0.0, 0.0, 0.0}, {0.0, 1.0, 0.0}}), std::invalid_argument);
}

TEST(Path, LoadsStraightRowReference) {
    const auto p = fp::Path::fromCsv(std::string(FIELDNAV_DATA_DIR) + "/reference_straight.csv");
    EXPECT_EQ(p.points().size(), 501U);
    EXPECT_DOUBLE_EQ(p.length(), 50.0);
}

TEST(Path, ClosestIndexOnStraightLine) {
    auto p = straight();
    EXPECT_EQ(p.closestIndex(Pose2D{3.04, 0.2, 0.0}), 30U);
}

TEST(Path, ClosestIndexFollowsTheRobotLocally) {
    auto p = straight();
    for (int k = 0; k <= 90; ++k) {
        const double x = 0.1 * k + 0.01;
        EXPECT_EQ(p.closestIndex(Pose2D{x, 0.4, 0.0}), static_cast<std::size_t>(k)) << "x = " << x;
    }
}

// ─── Erreur latérale (à écrire par Gil) ───

TEST(Path, LateralErrorIsZeroOnTheLine) {
    auto p = straight();
    EXPECT_NEAR(p.lateralError(Pose2D{4.37, 0.0, 0.0}), 0.0, 1e-12);
}

TEST(Path, LateralErrorIsPositiveOnTheLeft) {
    auto p = straight();
    EXPECT_NEAR(p.lateralError(Pose2D{5.0, 0.3, 0.0}), 0.3, 1e-12);
}

TEST(Path, LateralErrorIsNegativeOnTheRight) {
    auto p = straight();
    EXPECT_NEAR(p.lateralError(Pose2D{5.05, -0.3, 0.0}), -0.3, 1e-12);
}

TEST(Path, LateralErrorIgnoresHeading) {
    auto p = straight();
    EXPECT_NEAR(p.lateralError(Pose2D{5.0, 0.3, 1.2}), 0.3, 1e-12);
}

TEST(Path, LateralErrorOnACurve) {
    // Arc de rayon 10 m tournant à gauche : un robot à 9,5 m du centre est 0,5 m à GAUCHE.
    // Tolérance 1 mm : la corde entre deux points distants de 10 cm s'écarte de l'arc de
    // 0,1² / (8 × 10) ≈ 0,1 mm.
    auto p = arc(10.0);
    const double a = 0.6; // angle parcouru sur l'arc (rad)
    const Pose2D inside{9.5 * std::sin(a), 10.0 - 9.5 * std::cos(a), 0.0};
    EXPECT_NEAR(p.lateralError(inside), 0.5, 1e-3);
    const Pose2D outside{10.4 * std::sin(a), 10.0 - 10.4 * std::cos(a), 0.0};
    EXPECT_NEAR(p.lateralError(outside), -0.4, 1e-3);
}

// ─── Point visé (à écrire par Gil) ───

TEST(Path, LookaheadOneMetreAheadOnStraightLine) {
    auto p = straight();
    const auto q = p.lookahead(Pose2D{2.0, 0.3, 0.0}, 1.0);
    EXPECT_NEAR(q.x, 3.0, 1e-9);
    EXPECT_NEAR(q.y, 0.0, 1e-9);
}

TEST(Path, LookaheadInterpolatesBetweenPoints) {
    auto p = straight();
    const auto q = p.lookahead(Pose2D{2.0, 0.0, 0.0}, 0.25);
    EXPECT_NEAR(q.x, 2.25, 1e-9);
}

TEST(Path, LookaheadStopsAtTheEnd) {
    auto p = straight(10.0);
    const auto q = p.lookahead(Pose2D{9.5, 0.1, 0.0}, 2.0);
    EXPECT_NEAR(q.x, 10.0, 1e-9);
    EXPECT_NEAR(q.y, 0.0, 1e-9);
}

TEST(Path, LookaheadFollowsTheCurve) {
    // Sur l'arc, le point visé à d m le long du chemin est à l'angle (s + d)/R.
    auto p = arc(10.0);
    const auto q = p.lookahead(Pose2D{10.0 * std::sin(0.3), 10.0 * (1 - std::cos(0.3)), 0.0}, 2.0);
    const double a = 0.3 + 2.0 / 10.0;
    EXPECT_NEAR(q.x, 10.0 * std::sin(a), 2e-3);
    EXPECT_NEAR(q.y, 10.0 * (1 - std::cos(a)), 2e-3);
}
