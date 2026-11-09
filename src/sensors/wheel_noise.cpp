// Bruit multiplicatif des vitesses de roue.

#include <fieldnav/sensors/wheel_noise.hpp>

namespace fieldnav::sensors {

WheelNoise::WheelNoise(std::uint32_t seed, double sigma)
    : sigma_(sigma), rng_(seed), unit_normal_(0.0, 1.0) {}

double WheelNoise::apply(double true_speed) {
    const double eps = sigma_ * unit_normal_(rng_);
    return true_speed * (1.0 + eps);
}

} // namespace fieldnav::sensors
