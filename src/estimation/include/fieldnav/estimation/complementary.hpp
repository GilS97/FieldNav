// Fusion complémentaire odométrie + GPS : l'odométrie prédit la pose à chaque pas (lisse mais
// dérive), chaque mesure GPS la tire d'une fraction α vers la position mesurée (bruitée mais
// sans dérive). Rien à écrire dans ce fichier : le code va dans src/estimation/complementary.cpp.
#pragma once

#include <fieldnav/odometry/odometry.hpp>
#include <fieldnav/sensors/gps_sim.hpp>

namespace fieldnav::estimation {

// α par défaut, choisi le 9 nov. 2026 sur la vraie chaîne (GPS σ = 1 m, 5 Hz, 120 s avec virages,
// 5 seeds) : RMS de position 0,27 à 0,38 m, contre 1,8 à 2,2 m (odométrie) et 1,4 à 1,5 m (GPS).
// α = 0,1 à 0,3 (première idée de CLAUDE.md) recopie trop de bruit GPS : 0,35 à 0,65 m.
inline constexpr double kDefaultAlpha = 0.05;

class ComplementaryFuser {
public:
    explicit ComplementaryFuser(double alpha = kDefaultAlpha, const odometry::Pose2D& initial = {})
        : alpha_(alpha), odom_(initial) {}

    // Prédiction : un pas d'odométrie (v en m/s, w en rad/s, dt en s). Appelée à chaque pas.
    void predict(double v, double w, double dt) noexcept;

    // Correction : rapproche x et y de la mesure GPS d'une fraction alpha. Ne touche JAMAIS au
    // cap theta : le GPS ne mesure pas le cap. Appelée seulement quand une mesure arrive.
    void correct(const sensors::GpsFix& fix) noexcept;

    [[nodiscard]] const odometry::Pose2D& estimate() const noexcept { return odom_.pose(); }
    [[nodiscard]] double alpha() const noexcept { return alpha_; }

private:
    double alpha_;
    odometry::DiffDriveOdometry odom_; // porte la pose estimée
};

} // namespace fieldnav::estimation
