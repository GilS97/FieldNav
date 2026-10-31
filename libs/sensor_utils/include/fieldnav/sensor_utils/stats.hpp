// Statistiques sur des séries d'erreurs (erreur latérale, écart pose estimée / vérité terrain).
// Utilisées pour les RMS imprimés en fin de run, la métrique de succès et les légendes des figures.
#pragma once

#include <cmath>
#include <limits>
#include <span>

namespace fieldnav::sensor_utils {

// Racine de la moyenne des carrés : sqrt( (1/n) * somme(x_i^2) ).
// Série vide : renvoie NaN (pas de donnée, pas de valeur inventée ; « nan » reste visible dans le CSV).
[[nodiscard]] inline double rms(std::span<const double> values) noexcept {
    // TODO(Gil) : implémenter. Le stub renvoie NaN pour que les tests échouent tant que ce n'est pas fait.
    (void)values;
    return std::numeric_limits<double>::quiet_NaN();
}

// Plus grande valeur absolue de la série : max |x_i|.
// Série vide : renvoie NaN, comme rms().
[[nodiscard]] inline double maxAbs(std::span<const double> values) noexcept {
    // TODO(Gil) : implémenter.
    (void)values;
    return std::numeric_limits<double>::quiet_NaN();
}

} // namespace fieldnav::sensor_utils
