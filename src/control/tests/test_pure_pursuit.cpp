#include <gtest/gtest.h>

#include <cmath>
#include <fieldnav/control/pure_pursuit.hpp>
#include <vector>

namespace fc = fieldnav::control;
namespace fp = fieldnav::path;
using fieldnav::odometry::Pose2D;

namespace {

fp::Path straight(double length = 20.0) {
    std::vector<fp::RefPoint> pts;
    const auto n = static_cast<int>(std::lround(length / 0.1));
    for (int i = 0; i <= n; ++i) {
        const double s = i * 0.1;
        pts.push_back({s, s, 0.0});
    }
    return fp::Path(pts);
}

fp::Path arc(double radius) {
    std::vector<fp::RefPoint> pts;
    const auto n = static_cast<int>(std::lround(radius * 1.5 / 0.1));
    for (int i = 0; i <= n; ++i) {
        const double s = i * 0.1;
        pts.push_back({s, radius * std::sin(s / radius), radius * (1.0 - std::cos(s / radius))});
    }
    return fp::Path(pts);
}

} // namespace

TEST(PurePursuit, OnTheLineAndAlignedGoesStraight) {
    auto p = straight();
    const auto c = fc::purePursuit(Pose2D{2.0, 0.0, 0.0}, p);
    EXPECT_DOUBLE_EQ(c.v, 0.3);
    EXPECT_NEAR(c.w, 0.0, 1e-12);
}

TEST(PurePursuit, LeftOfTheLineTurnsRight) {
    // Test géométrique de la tâche : robot 1 m à gauche, cap parallèle → ω négatif.
    auto p = straight();
    EXPECT_LT(fc::purePursuit(Pose2D{2.0, 1.0, 0.0}, p).w, 0.0);
}

TEST(PurePursuit, RightOfTheLineTurnsLeft) {
    auto p = straight();
    EXPECT_GT(fc::purePursuit(Pose2D{2.0, -0.3, 0.0}, p).w, 0.0);
}

TEST(PurePursuit, HeadingTooFarLeftTurnsRight) {
    auto p = straight();
    EXPECT_LT(fc::purePursuit(Pose2D{2.0, 0.0, 0.3}, p).w, 0.0);
}

TEST(PurePursuit, CurvatureFormulaMatchesGeometry) {
    // Robot 0,2 m à gauche, cap parallèle, lookahead 0,8 m : point visé en (2,8 ; 0).
    // α = atan2(−0,2 ; 0,8), L_d = √(0,8² + 0,2²) → ω = v · 2 sin α / L_d.
    auto p = straight();
    fc::PurePursuitParams params;
    const auto c = fc::purePursuit(Pose2D{2.0, 0.2, 0.0}, p, params);
    const double alpha = std::atan2(-0.2, 0.8);
    const double ld = std::hypot(0.8, 0.2);
    EXPECT_NEAR(c.w, 0.3 * 2.0 * std::sin(alpha) / ld, 1e-6);
}

TEST(PurePursuit, SaturatesTurnRate) {
    // Robot sur la ligne mais tourné de 90° vers la gauche, lookahead court : la formule
    // demande ω = 0,3 · 2 · sin(−π/2) / 0,3 = −2 rad/s, ramené à −w_max = −1,5 rad/s.
    auto p = straight();
    fc::PurePursuitParams params;
    params.lookahead = 0.3;
    const auto c = fc::purePursuit(Pose2D{2.0, 0.0, 1.5707963267948966}, p, params);
    EXPECT_DOUBLE_EQ(c.w, -params.w_max);
}

TEST(PurePursuit, FollowsArcWithItsCurvature) {
    // Sur un arc de rayon 10 m, posé sur la ligne et tangent : ω ≈ v / R = 0,03 rad/s.
    auto p = arc(10.0);
    const double a = 0.4;
    const auto c = fc::purePursuit(Pose2D{10.0 * std::sin(a), 10.0 * (1.0 - std::cos(a)), a}, p);
    EXPECT_NEAR(c.w, 0.3 / 10.0, 1e-3);
}

TEST(PurePursuit, StopsAtTheEndOfThePath) {
    auto p = straight(5.0);
    const auto c = fc::purePursuit(Pose2D{5.0, 0.0, 0.0}, p);
    EXPECT_DOUBLE_EQ(c.v, 0.0);
    EXPECT_DOUBLE_EQ(c.w, 0.0);
}
