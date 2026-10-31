// Statistiques sur des séries d'erreurs (erreur latérale, écart pose estimée / vérité terrain).
// Utilisées pour les RMS imprimés en fin de run, la métrique de succès et les légendes des figures.
//
// ─── À CODER (Gil) ─────────────────────────────────────────────────────────────────────────────
// Deux fonctions à écrire ci-dessous, à la place des deux stubs marqués TODO(Gil).
// Test qui valide : libs/sensor_utils/tests/test_stats.cpp (9 tests, 7 échouent tant que les stubs
// sont là). Commande :
//   cmake --build build/debug && ctest --test-dir build/debug --output-on-failure
//
// Règles communes aux deux fonctions :
//   - Série vide (values.empty()) : renvoyer NaN immédiatement, c'est la première ligne à écrire.
//   - Parcourir avec `for (const double x : values)`.
//     Pas de `for (int i = 0; i < values.size(); ++i)` : comparaison int / size_t
//     → -Wsign-conversion, qui est une erreur dans la CI.
//   - Fonctions `noexcept` : pas d'exception, pas d'allocation (pas de std::vector temporaire).
//   - Includes déjà présents : <cmath> (std::sqrt, std::abs), <limits> (quiet_NaN), <span>.
// ───────────────────────────────────────────────────────────────────────────────────────────────
#pragma once

#include <cmath>
#include <limits>
#include <span>

namespace fieldnav::sensor_utils {

// Racine de la moyenne des carrés : sqrt( (1/n) * somme(x_i^2) ).
// Série vide : renvoie NaN (pas de donnée, pas de valeur inventée ; « nan » reste visible
// dans le CSV).
[[nodiscard]] inline double rms(std::span<const double> values) noexcept {
    // TODO(Gil) : remplacer les 2 lignes du stub par :
    //   1. si values est vide → return std::numeric_limits<double>::quiet_NaN();
    //   2. double sum_sq = 0.0;  puis pour chaque x : sum_sq += x * x;
    //   3. diviser par n : n vaut values.size() (type size_t). Convertir explicitement avec
    //      static_cast<double>(values.size()), sinon -Wconversion (erreur en CI).
    //   4. return std::sqrt(sum_sq / n);
    // Vérif mentale : {3, 4} → (9 + 16) / 2 = 12,5 → sqrt = 3,5355.

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
    // TODO(Gil) : remplacer les 2 lignes du stub par :
    //   1. si values est vide → return NaN (comme rms).
    //   2. double m = 0.0;  puis pour chaque x : si std::abs(x) > m alors m = std::abs(x);
    //      (0.0 est un bon départ car une valeur absolue n'est jamais négative.)
    //   3. return m;
    // Piège : std::abs et pas abs tout court (abs(int) tronquerait -0,7 en 0).
    // Vérif mentale : {1, -5, 3} → 5.
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
