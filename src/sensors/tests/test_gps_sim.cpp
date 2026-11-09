#include <gtest/gtest.h>

#include <cmath>
#include <fieldnav/sensors/gps_sim.hpp>
#include <vector>

using fieldnav::sensors::GpsFix;
using fieldnav::sensors::GpsParams;
using fieldnav::sensors::GpsSim;

namespace {

constexpr double kDt = 0.002; // pas de la simulation MuJoCo

// Appelle update() pendant `seconds` (pas de 2 ms), robot immobile en (x, y).
std::vector<GpsFix> run(GpsSim& gps, double seconds, double x = 0.0, double y = 0.0) {
    std::vector<GpsFix> fixes;
    const auto steps = static_cast<int>(std::lround(seconds / kDt));
    for (int i = 1; i <= steps; ++i) {
        if (const auto f = gps.update(i * kDt, x, y)) {
            fixes.push_back(*f);
        }
    }
    return fixes;
}

GpsParams perfect() {
    GpsParams p;
    p.sigma_noise = 0.0;
    p.sigma_rw = 0.0;
    p.p_loss = 0.0;
    return p;
}

} // namespace

// ─── Planning des mesures (déjà écrit, passe tout de suite) ───

TEST(GpsSim, FiveFixesPerSecond) {
    GpsSim gps(1, perfect());
    EXPECT_EQ(run(gps, 10.0).size(), 50U);
}

TEST(GpsSim, OutageRemovesFixesInsideWindowOnly) {
    GpsParams p = perfect();
    p.outage_start = 10.0;
    p.outage_end = 20.0;
    GpsSim gps(1, p);
    const auto fixes = run(gps, 30.0);
    bool before = false, after = false;
    for (const auto& f : fixes) {
        EXPECT_FALSE(f.t >= 10.0 && f.t <= 20.0) << "mesure pendant le trou à t = " << f.t;
        before = before || f.t < 10.0;
        after = after || f.t > 20.0;
    }
    EXPECT_TRUE(before);
    EXPECT_TRUE(after);
}

// ─── Modèle de mesure (à écrire par Gil) ───

TEST(GpsSim, PerfectGpsGivesTruth) {
    GpsSim gps(3, perfect());
    for (const auto& f : run(gps, 5.0, 4.0, -1.5)) {
        EXPECT_DOUBLE_EQ(f.x, 4.0);
        EXPECT_DOUBLE_EQ(f.y, -1.5);
    }
}

TEST(GpsSim, WhiteNoiseHasSigmaNoise) {
    // 10 000 mesures autour de (0, 0), sans biais ni perte : écart-type ≈ 1 m (à 10 % près).
    GpsParams p = perfect();
    p.sigma_noise = 1.0;
    GpsSim gps(7, p);
    const auto fixes = run(gps, 2000.0);
    ASSERT_EQ(fixes.size(), 10000U);
    double sum = 0.0, sum_sq = 0.0;
    for (const auto& f : fixes) {
        sum += f.x;
        sum_sq += f.x * f.x;
    }
    const double n = static_cast<double>(fixes.size());
    const double mean = sum / n;
    EXPECT_NEAR(mean, 0.0, 0.05); // écart-type de la moyenne : 1/√10000 = 0,01 m
    EXPECT_NEAR(std::sqrt(sum_sq / n - mean * mean), 1.0, 0.1);
}

TEST(GpsSim, LossRateMatchesProbability) {
    GpsParams p = perfect();
    p.p_loss = 0.2;
    GpsSim gps(11, p);
    const auto received = static_cast<double>(run(gps, 2000.0).size());
    EXPECT_NEAR(1.0 - received / 10000.0, 0.2, 0.02);
}

TEST(GpsSim, BiasRandomWalkGrowsLikeSqrtTime) {
    // 200 GPS indépendants, 100 s chacun : l'écart-type du biais final ≈ 0,02·√100 = 0,2 m.
    GpsParams p = perfect();
    p.sigma_rw = 0.02;
    double sum_sq = 0.0;
    const int runs = 200;
    for (int s = 0; s < runs; ++s) {
        GpsSim gps(static_cast<std::uint32_t>(1000 + s), p);
        (void)run(gps, 100.0);
        sum_sq += gps.biasX() * gps.biasX();
    }
    EXPECT_NEAR(std::sqrt(sum_sq / runs), 0.2, 0.03);
}

TEST(GpsSim, BiasIsSlowComparedToNoise) {
    // Entre deux mesures (0,2 s), le biais ne bouge que de 0,02·√0,2 ≈ 9 mm : c'est une erreur
    // lente, que le filtrage ne peut pas moyenner.
    GpsParams p = perfect();
    p.sigma_rw = 0.02;
    GpsSim gps(5, p);
    double previous = 0.0, max_jump = 0.0;
    for (int i = 1; i <= 5000; ++i) {
        (void)gps.update(i * kDt, 0.0, 0.0);
        max_jump = std::fmax(max_jump, std::fabs(gps.biasX() - previous));
        previous = gps.biasX();
    }
    EXPECT_GT(max_jump, 0.0);  // échoue tant que stepBias() ne fait rien
    EXPECT_LT(max_jump, 0.05); // 5 σ d'un pas : aucun saut brutal
}

// ─── Reproductibilité ───

TEST(GpsSim, SameSeedSameFixes) {
    GpsSim a(42), b(42);
    const auto fa = run(a, 20.0, 3.0, 1.0);
    const auto fb = run(b, 20.0, 3.0, 1.0);
    ASSERT_EQ(fa.size(), fb.size());
    for (std::size_t i = 0; i < fa.size(); ++i) {
        EXPECT_DOUBLE_EQ(fa[i].x, fb[i].x);
        EXPECT_DOUBLE_EQ(fa[i].y, fb[i].y);
    }
}

TEST(GpsSim, DifferentSeedsDiffer) {
    GpsSim a(1), b(2);
    const auto fa = run(a, 5.0), fb = run(b, 5.0);
    ASSERT_FALSE(fa.empty());
    ASSERT_FALSE(fb.empty());
    EXPECT_NE(fa.front().x, fb.front().x);
}

TEST(GpsSim, OutageDoesNotShiftTheRandomSequence) {
    // Même seed, l'un avec un trou GNSS entre 2 et 4 s : après le trou, mêmes mesures.
    GpsParams with_outage;
    with_outage.outage_start = 2.0;
    with_outage.outage_end = 4.0;
    GpsSim a(9), b(9, with_outage);
    const auto fa = run(a, 10.0), fb = run(b, 10.0);
    std::vector<GpsFix> fa_after, fb_after;
    for (const auto& f : fa) {
        if (f.t > 4.0)
            fa_after.push_back(f);
    }
    for (const auto& f : fb) {
        if (f.t > 4.0)
            fb_after.push_back(f);
    }
    ASSERT_EQ(fa_after.size(), fb_after.size());
    ASSERT_FALSE(fa_after.empty());
    for (std::size_t i = 0; i < fa_after.size(); ++i) {
        EXPECT_DOUBLE_EQ(fa_after[i].x, fb_after[i].x);
    }
}
