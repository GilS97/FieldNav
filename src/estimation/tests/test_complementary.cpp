#include <gtest/gtest.h>

#include <cmath>
#include <fieldnav/estimation/complementary.hpp>

namespace est = fieldnav::estimation;
namespace odo = fieldnav::odometry;
using fieldnav::sensors::GpsFix;

TEST(ComplementaryFuser, DefaultAlphaIsDocumentedValue) {
    const est::ComplementaryFuser f;
    EXPECT_DOUBLE_EQ(f.alpha(), 0.05);
}

TEST(ComplementaryFuser, NoGpsEqualsOdometry) {
    // Sans mesure GPS, la fusion doit donner exactement la même pose que l'odométrie seule.
    est::ComplementaryFuser f;
    odo::DiffDriveOdometry o;
    for (int i = 0; i < 1000; ++i) {
        const double w = 0.5 * std::sin(i * 0.01);
        f.predict(0.3, w, 0.002);
        o.update(0.3, w, 0.002);
    }
    EXPECT_GT(o.pose().x, 0.5); // échoue tant que predict() ne fait rien
    EXPECT_DOUBLE_EQ(f.estimate().x, o.pose().x);
    EXPECT_DOUBLE_EQ(f.estimate().y, o.pose().y);
    EXPECT_DOUBLE_EQ(f.estimate().theta, o.pose().theta);
}

TEST(ComplementaryFuser, CorrectionMovesFractionAlphaTowardFix) {
    est::ComplementaryFuser f(0.2);
    f.correct(GpsFix{0.2, 1.0, 2.0});
    EXPECT_NEAR(f.estimate().x, 0.2, 1e-12);
    EXPECT_NEAR(f.estimate().y, 0.4, 1e-12);
}

TEST(ComplementaryFuser, CorrectionNeverTouchesHeading) {
    est::ComplementaryFuser f(0.3, odo::Pose2D{0.0, 0.0, 1.2});
    f.correct(GpsFix{0.2, 5.0, -3.0});
    EXPECT_DOUBLE_EQ(f.estimate().theta, 1.2);
    EXPECT_NE(f.estimate().x, 0.0); // la correction a bien eu lieu
}

TEST(ComplementaryFuser, AlphaZeroIgnoresGpsAlphaOneCopiesIt) {
    est::ComplementaryFuser keep(0.0), copy(1.0);
    keep.correct(GpsFix{0.2, 3.0, 4.0});
    copy.correct(GpsFix{0.2, 3.0, 4.0});
    EXPECT_DOUBLE_EQ(keep.estimate().x, 0.0);
    EXPECT_DOUBLE_EQ(copy.estimate().x, 3.0);
    EXPECT_DOUBLE_EQ(copy.estimate().y, 4.0);
}

TEST(ComplementaryFuser, PerfectGpsConvergesGeometrically) {
    // Estimée fausse de 1 m, GPS parfait : après n corrections, l'erreur vaut (1 − α)^n.
    est::ComplementaryFuser f(0.05, odo::Pose2D{1.0, 0.0, 0.0});
    for (int n = 0; n < 100; ++n) {
        f.correct(GpsFix{0.2 * n, 0.0, 0.0});
    }
    EXPECT_NEAR(f.estimate().x, std::pow(0.95, 100), 1e-12); // ≈ 0,6 %
}
