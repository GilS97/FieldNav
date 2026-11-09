// Bruit des capteurs de roue : chaque vitesse mesurée est faussée de quelques pourcents.
// Rien à écrire dans ce fichier : le code va dans src/sensors/wheel_noise.cpp.
//
// Reproductible (CLAUDE.md) : tout l'aléa vient d'un std::mt19937 seedé, interne à l'objet.
// Même seed + mêmes appels = mêmes valeurs, octet pour octet, sur une même plateforme.
// (std::normal_distribution peut différer entre bibliothèques standard : libstdc++ ≠ MSVC.)
#pragma once

#include <cstdint>
#include <random>

namespace fieldnav::sensors {

class WheelNoise {
public:
    // sigma : écart-type relatif du bruit (0,02 = ±2 %). sigma = 0 → aucune perturbation.
    explicit WheelNoise(std::uint32_t seed, double sigma = 0.02);

    // Vitesse de roue mesurée = vitesse vraie × (1 + ε), ε tiré selon N(0, sigma²).
    // Bruit multiplicatif : une roue arrêtée reste mesurée à 0 (pas de dérive à l'arrêt).
    [[nodiscard]] double apply(double true_speed);

    [[nodiscard]] double sigma() const noexcept { return sigma_; }

private:
    double sigma_;
    std::mt19937 rng_;
    // Loi normale STANDARD N(0, 1) ; ε = sigma × tirage. (Une loi N(0, sigma²) directe est
    // interdite pour sigma = 0 : la norme C++ exige un écart-type strictement positif.)
    std::normal_distribution<double> unit_normal_;
};

} // namespace fieldnav::sensors
