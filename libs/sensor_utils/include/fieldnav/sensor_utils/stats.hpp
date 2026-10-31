// Statistiques sur des séries d'erreurs (erreur latérale, écart pose estimée / vérité terrain).
// Utilisées pour les RMS imprimés en fin de run, la métrique de succès et les légendes des figures.
//
#pragma once

#include <cmath>
#include <limits>
#include <span>

namespace fieldnav::sensor_utils {

// Racine de la moyenne des carrés : sqrt( (1/n) * somme(x_i^2) ).
// Série vide : renvoie NaN (pas de donnée, pas de valeur inventée ; « nan » reste visible
// dans le CSV).
[[nodiscard]] inline double rms(std::span<const double> values) noexcept {

    if (values.empty()) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    double sum_sq = 0.0;
    for (const double x : values) {
        sum_sq += x * x;
    }
    return std::sqrt(sum_sq / static_cast<double>(values.size()));
}

// Plus grande valeur absolue de la série : max |x_i|.
// Série vide : renvoie NaN, comme rms().
[[nodiscard]] inline double maxAbs(std::span<const double> values) noexcept {
    if (values.empty()) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    double m = 0.0;
    for (const double x : values) {
        if (std::abs(x) > m) {
            m = std::abs(x);
        }
    }
    return m;
}

} // namespace fieldnav::sensor_utils
