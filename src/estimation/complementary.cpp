// Fusion complémentaire odométrie + GPS.

#include <fieldnav/estimation/complementary.hpp>

namespace fieldnav::estimation {

void ComplementaryFuser::predict(double v, double w, double dt) noexcept {
    odom_.update(v, w, dt);
}

void ComplementaryFuser::correct(const sensors::GpsFix& fix) noexcept {
    odometry::Pose2D p = odom_.pose();
    p.x += alpha_ * (fix.x - p.x);
    p.y += alpha_ * (fix.y - p.y);
    odom_.reset(p);
}

} // namespace fieldnav::estimation
