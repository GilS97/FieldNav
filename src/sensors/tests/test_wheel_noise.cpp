#include <gtest/gtest.h>

#include <cmath>
#include <fieldnav/sensors/wheel_noise.hpp>

using fieldnav::sensors::WheelNoise;

TEST(WheelNoise, ZeroSigmaIsExact) {
    WheelNoise noise(42, 0.0);
    for (int i = 0; i < 100; ++i) {
        EXPECT_DOUBLE_EQ(noise.apply(10.0), 10.0);
    }
}

TEST(WheelNoise, StoppedWheelStaysAtZero) {
    WheelNoise noise(42);
    EXPECT_DOUBLE_EQ(noise.apply(0.0), 0.0);
}

TEST(WheelNoise, ActuallyPerturbs) {
    WheelNoise noise(42);
    int different = 0;
    for (int i = 0; i < 100; ++i) {
        if (noise.apply(10.0) != 10.0) {
            ++different;
        }
    }
    EXPECT_EQ(different, 100); // échoue tant que apply() renvoie la vitesse vraie
}

TEST(WheelNoise, StatisticsMatchSigma) {
    // 10 000 mesures de 10 rad/s : moyenne ≈ 10, écart-type relatif ≈ 2 % (à 10 % près).
    WheelNoise noise(7, 0.02);
    const int n = 10000;
    double sum = 0.0;
    double sum_sq = 0.0;
    for (int i = 0; i < n; ++i) {
        const double rel = noise.apply(10.0) / 10.0 - 1.0;
        sum += rel;
        sum_sq += rel * rel;
    }
    const double mean = sum / n;
    const double stddev = std::sqrt(sum_sq / n - mean * mean);
    EXPECT_NEAR(mean, 0.0, 0.001);
    EXPECT_NEAR(stddev, 0.02, 0.002);
}

TEST(WheelNoise, SameSeedSameSequence) {
    WheelNoise a(123);
    WheelNoise b(123);
    for (int i = 0; i < 1000; ++i) {
        EXPECT_DOUBLE_EQ(a.apply(5.0), b.apply(5.0));
    }
}

TEST(WheelNoise, DifferentSeedsDiffer) {
    WheelNoise a(1);
    WheelNoise b(2);
    EXPECT_NE(a.apply(5.0), b.apply(5.0));
}

TEST(WheelNoise, DrawsEvenWhenStopped) {
    // Deux objets de même seed : l'un mesure une roue arrêtée, l'autre une roue qui tourne.
    // Le tirage suivant doit rester identique (l'arrêt consomme aussi un nombre aléatoire).
    WheelNoise a(99);
    WheelNoise b(99);
    (void)a.apply(0.0);
    (void)b.apply(8.0);
    EXPECT_DOUBLE_EQ(a.apply(3.0), b.apply(3.0));
}
