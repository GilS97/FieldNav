// Cinématique du robot différentiel : vitesses de roues ↔ (v, ω).

#include <fieldnav/odometry/kinematics.hpp>

namespace fieldnav::odometry {

Twist wheelsToTwist(const WheelRates& wheels, double radius, double track) noexcept {
    const double v = radius * (wheels.right + wheels.left) / 2.0;
    const double w = radius * (wheels.right - wheels.left) / track;
    return Twist{v, w};
}

WheelRates twistToWheels(const Twist& twist, double radius, double track) noexcept {
    const double left = (twist.v - twist.w * track / 2.0) / radius;
    const double right = (twist.v + twist.w * track / 2.0) / radius;
    return WheelRates{left, right};
}

} // namespace fieldnav::odometry
