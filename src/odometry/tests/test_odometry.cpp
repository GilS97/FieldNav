#include <gtest/gtest.h>

#include <cmath>
#include <fieldnav/odometry/odometry.hpp>

namespace odo = fieldnav::odometry;

namespace {
constexpr double kPi = 3.14159265358979323846;
}

// ─── normalizeAngle ───

TEST(NormalizeAngle, KeepsAnglesAlreadyInRange) {
    EXPECT_NEAR(odo::normalizeAngle(0.0), 0.0, 1e-12);
    EXPECT_NEAR(odo::normalizeAngle(1.0), 1.0, 1e-12);
    EXPECT_NEAR(odo::normalizeAngle(-2.0), -2.0, 1e-12);
}

TEST(NormalizeAngle, WrapsOneTurn) {
    EXPECT_NEAR(odo::normalizeAngle(1.5 * kPi), -0.5 * kPi, 1e-12);
    EXPECT_NEAR(odo::normalizeAngle(-1.5 * kPi), 0.5 * kPi, 1e-12);
}

TEST(NormalizeAngle, WrapsManyTurns) {
    EXPECT_NEAR(odo::normalizeAngle(10.0 * kPi + 0.1), 0.1, 1e-9);
    EXPECT_NEAR(odo::normalizeAngle(-10.0 * kPi - 0.1), -0.1, 1e-9);
}

TEST(NormalizeAngle, AlwaysInRange) {
    for (int i = -100; i <= 100; ++i) {
        const double a = odo::normalizeAngle(0.37 * i);
        EXPECT_GE(a, -kPi);
        EXPECT_LE(a, kPi);
    }
}

// ─── DiffDriveOdometry ───

TEST(DiffDriveOdometry, StartsAtOriginByDefault) {
    const odo::DiffDriveOdometry odom;
    EXPECT_DOUBLE_EQ(odom.pose().x, 0.0);
    EXPECT_DOUBLE_EQ(odom.pose().y, 0.0);
    EXPECT_DOUBLE_EQ(odom.pose().theta, 0.0);
}

TEST(DiffDriveOdometry, StraightLine) {
    odo::DiffDriveOdometry odom;
    for (int i = 0; i < 10; ++i) {
        odom.update(1.0, 0.0, 1.0);
    }
    EXPECT_NEAR(odom.pose().x, 10.0, 1e-9);
    EXPECT_NEAR(odom.pose().y, 0.0, 1e-9);
    EXPECT_NEAR(odom.pose().theta, 0.0, 1e-12);
}

TEST(DiffDriveOdometry, BackwardMovesNegativeX) {
    odo::DiffDriveOdometry odom;
    odom.update(-0.5, 0.0, 2.0);
    EXPECT_NEAR(odom.pose().x, -1.0, 1e-12);
}

TEST(DiffDriveOdometry, QuarterTurnInPlace) {
    odo::DiffDriveOdometry odom;
    for (int i = 0; i < 100; ++i) {
        odom.update(0.0, kPi / 2.0, 0.01);
    }
    EXPECT_NEAR(odom.pose().theta, kPi / 2.0, 1e-9);
    EXPECT_NEAR(odom.pose().x, 0.0, 1e-12);
    EXPECT_NEAR(odom.pose().y, 0.0, 1e-12);
}

TEST(DiffDriveOdometry, MovesAlongCurrentHeading) {
    // Cap initial à 90° : avancer de 2 m fait monter y, pas x.
    odo::DiffDriveOdometry odom(odo::Pose2D{1.0, 1.0, kPi / 2.0});
    odom.update(1.0, 0.0, 2.0);
    EXPECT_NEAR(odom.pose().x, 1.0, 1e-9);
    EXPECT_NEAR(odom.pose().y, 3.0, 1e-9);
}

TEST(DiffDriveOdometry, CircleReturnsNearOrigin) {
    // v = 1 m/s, ω = 1 rad/s pendant 2π s : un cercle de rayon 1 m. Euler dérive un peu ;
    // tolérance de 5 cm avec dt = 1e-3 (voir CLAUDE.md, ne pas changer d'intégrateur).
    odo::DiffDriveOdometry odom;
    const double dt = 1e-3;
    const auto steps = static_cast<int>(std::lround(2.0 * kPi / dt));
    for (int i = 0; i < steps; ++i) {
        odom.update(1.0, 1.0, dt);
    }
    EXPECT_NEAR(odom.pose().x, 0.0, 0.05);
    EXPECT_NEAR(odom.pose().y, 0.0, 0.05);
    EXPECT_NEAR(odom.pose().theta, 0.0, 0.01);
}

TEST(DiffDriveOdometry, HeadingStaysNormalizedAfterManyTurns) {
    odo::DiffDriveOdometry odom;
    for (int i = 0; i < 10000; ++i) {
        odom.update(0.0, 1.0, 0.01); // 100 rad ≈ 16 tours
    }
    EXPECT_GE(odom.pose().theta, -kPi);
    EXPECT_LE(odom.pose().theta, kPi);
}

TEST(DiffDriveOdometry, ResetRestoresPose) {
    odo::DiffDriveOdometry odom;
    odom.update(1.0, 0.5, 1.0);
    odom.reset();
    EXPECT_DOUBLE_EQ(odom.pose().x, 0.0);
    EXPECT_DOUBLE_EQ(odom.pose().theta, 0.0);
}
