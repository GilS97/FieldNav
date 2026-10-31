#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <fieldnav/sensor_utils/stats.hpp>
#include <vector>

// sensor_utils doit propager cxx_std_20 aux cibles qui le lient (le reste de FieldNav : C++17).
static_assert(__cplusplus >= 202002L, "fieldnav::sensor_utils doit propager cxx_std_20");

namespace su = fieldnav::sensor_utils;

TEST(Rms, KnownValues) {
    const std::vector<double> v{3.0, 4.0};
    EXPECT_NEAR(su::rms(v), std::sqrt(12.5), 1e-12);
}

TEST(Rms, SignDoesNotMatter) {
    const std::vector<double> v{-2.0, 2.0, -2.0, 2.0};
    EXPECT_NEAR(su::rms(v), 2.0, 1e-12);
}

TEST(Rms, ConstantSeriesEqualsItsAbsoluteValue) {
    // 50 m à 10 cm d'écart constant : RMS = 10 cm.
    const std::vector<double> v(500, -0.1);
    EXPECT_NEAR(su::rms(v), 0.1, 1e-12);
}

TEST(Rms, AcceptsStdArray) {
    const std::array<double, 1> a{0.15};
    EXPECT_NEAR(su::rms(a), 0.15, 1e-12);
}

TEST(Rms, EmptyIsNaN) {
    const std::vector<double> v;
    EXPECT_TRUE(std::isnan(su::rms(v)));
}

TEST(MaxAbs, PicksLargestMagnitude) {
    const std::vector<double> v{1.0, -5.0, 3.0};
    EXPECT_DOUBLE_EQ(su::maxAbs(v), 5.0);
}

TEST(MaxAbs, AllNegative) {
    const std::vector<double> v{-0.3, -0.7, -0.2};
    EXPECT_DOUBLE_EQ(su::maxAbs(v), 0.7);
}

TEST(MaxAbs, EmptyIsNaN) {
    const std::vector<double> v;
    EXPECT_TRUE(std::isnan(su::maxAbs(v)));
}

TEST(Stats, MaxAbsIsAtLeastRms) {
    const std::vector<double> v{0.02, -0.11, 0.07, -0.04, 0.09};
    EXPECT_GE(su::maxAbs(v), su::rms(v));
}
