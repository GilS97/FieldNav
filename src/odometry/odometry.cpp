// Odométrie différentielle : implémentation de normalizeAngle() et DiffDriveOdometry::update().
#include <cmath>
#include <fieldnav/odometry/odometry.hpp>

namespace fieldnav::odometry {

double normalizeAngle(double angle) noexcept {
    return std::atan2(std::sin(angle), std::cos(angle));
}

void DiffDriveOdometry::update(double v, double w, double dt) noexcept {

    pose_.x += v * std::cos(pose_.theta) * dt;
    pose_.y += v * std::sin(pose_.theta) * dt;
    pose_.theta += w * dt;
    pose_.theta = normalizeAngle(pose_.theta);

    (void)v;
    (void)w;
    (void)dt;
}

} // namespace fieldnav::odometry
