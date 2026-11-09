// GPS simulé : biais en marche aléatoire + bruit blanc + pertes + trou GNSS.

#include <cmath>
#include <fieldnav/sensors/gps_sim.hpp>

namespace fieldnav::sensors {

GpsSim::GpsSim(std::uint32_t seed, const GpsParams& params)
    : p_(params), rng_(seed), unit_normal_(0.0, 1.0), unit_uniform_(0.0, 1.0) {}

std::optional<GpsFix> GpsSim::update(double t, double true_x, double true_y) {
    // Instant du prochain échantillon, calculé depuis un compteur entier : pas de dérive
    // d'horloge par accumulation d'arrondis (0,2 + 0,2 + ... n'est pas exactement 0,2·n).
    const double period = 1.0 / p_.rate_hz;
    // Première mesure à t = période (0,2 s), comme un récepteur qui attend sa première époque.
    const double next_sample_time = static_cast<double>(sample_index_ + 1) * period;
    if (t + 1e-9 < next_sample_time) {
        return std::nullopt; // pas encore l'heure d'une mesure
    }
    ++sample_index_;

    // Les tirages sont faits dans le même ordre à chaque échantillon, même si la mesure est
    // ensuite jetée (perte ou trou) : la suite aléatoire ne dépend que du nombre d'échantillons.
    stepBias(period);
    const GpsFix fix = measure(t, true_x, true_y);
    const bool lost = isLost();

    const bool in_outage = p_.outage_start >= 0.0 && t >= p_.outage_start && t <= p_.outage_end;
    if (lost || in_outage) {
        return std::nullopt;
    }
    return fix;
}

void GpsSim::stepBias(double dt) {
    const double step = p_.sigma_rw * std::sqrt(dt);
    bias_x_ += step * unit_normal_(rng_);
    bias_y_ += step * unit_normal_(rng_);
}

GpsFix GpsSim::measure(double t, double x, double y) {
    const double noise_x = p_.sigma_noise * unit_normal_(rng_);
    const double noise_y = p_.sigma_noise * unit_normal_(rng_);
    return GpsFix{t, x + bias_x_ + noise_x, y + bias_y_ + noise_y};
}

bool GpsSim::isLost() {
    return unit_uniform_(rng_) < p_.p_loss;
}

} // namespace fieldnav::sensors
