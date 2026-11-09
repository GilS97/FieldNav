#include <gtest/gtest.h>

#include <fieldnav/odometry/kinematics.hpp>

namespace odo = fieldnav::odometry;

namespace {
constexpr double kRadius = 0.0325; // roue TT de 65 mm
constexpr double kTrack = 0.22;    // entraxe effectif du châssis 4 roues (mesuré en sim)
} // namespace

TEST(Kinematics, EqualWheelsGoStraight) {
    const auto t = odo::wheelsToTwist({10.0, 10.0}, kRadius, kTrack);
    EXPECT_NEAR(t.v, 0.325, 1e-12);
    EXPECT_NEAR(t.w, 0.0, 1e-12);
}

TEST(Kinematics, FasterRightWheelTurnsLeft) {
    const auto t = odo::wheelsToTwist({5.0, 10.0}, kRadius, kTrack);
    EXPECT_NEAR(t.v, kRadius * 7.5, 1e-12);
    EXPECT_GT(t.w, 0.0);
    EXPECT_NEAR(t.w, kRadius * 5.0 / kTrack, 1e-12);
}

TEST(Kinematics, OppositeWheelsSpinInPlace) {
    const auto t = odo::wheelsToTwist({-5.0, 5.0}, kRadius, kTrack);
    EXPECT_NEAR(t.v, 0.0, 1e-12);
    EXPECT_NEAR(t.w, kRadius * 10.0 / kTrack, 1e-12);
}

TEST(Kinematics, BackwardIsNegativeV) {
    const auto t = odo::wheelsToTwist({-4.0, -4.0}, kRadius, kTrack);
    EXPECT_NEAR(t.v, -0.13, 1e-12);
}

TEST(Kinematics, InverseOfStraightCommand) {
    const auto w = odo::twistToWheels({0.325, 0.0}, kRadius, kTrack);
    EXPECT_NEAR(w.left, 10.0, 1e-9);
    EXPECT_NEAR(w.right, 10.0, 1e-9);
}

TEST(Kinematics, InverseOfSpinLeft) {
    const auto w = odo::twistToWheels({0.0, 1.0}, kRadius, kTrack);
    EXPECT_LT(w.left, 0.0);
    EXPECT_GT(w.right, 0.0);
    EXPECT_NEAR(w.right, 0.11 / kRadius, 1e-9); // ω·L/2 / r
}

TEST(Kinematics, RoundTrip) {
    const odo::WheelRates in{3.7, -1.2};
    const auto out = odo::twistToWheels(odo::wheelsToTwist(in, kRadius, kTrack), kRadius, kTrack);
    EXPECT_NEAR(out.left, in.left, 1e-9);
    EXPECT_NEAR(out.right, in.right, 1e-9);
}

// Non-régression : valeurs figées le 2 nov. 2026 (r = 0,0325 m, L = 0,22 m).
TEST(Kinematics, RegressionValues) {
    const auto t = odo::wheelsToTwist({6.0, 9.0}, kRadius, kTrack);
    EXPECT_NEAR(t.v, 0.24375, 1e-12);
    EXPECT_NEAR(t.w, 0.443181818181818, 1e-12);
}
