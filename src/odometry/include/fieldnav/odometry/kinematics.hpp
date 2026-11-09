// Cinématique du robot différentiel : vitesses de roues ↔ (v, ω).
// Sert dans les deux sens : roues mesurées → (v, ω) pour l'odométrie ; (v, ω) commandés → roues
// pour piloter. Rien à écrire dans ce fichier : le code va dans src/odometry/kinematics.cpp.
#pragma once

namespace fieldnav::odometry {

// Vitesse du robot : v en m/s le long de son cap, w (ω) en rad/s (positif = vers la gauche).
struct Twist {
    double v{0.0};
    double w{0.0};
};

// Vitesses angulaires des roues de chaque côté, en rad/s (positif = fait avancer le robot).
struct WheelRates {
    double left{0.0};
    double right{0.0};
};

// Roues → (v, ω). radius : rayon de roue (m). track : entraxe (m).
// Pour le châssis 4 roues (skid-steer), passer l'entraxe EFFECTIF (≈ 0,22 m en sim), pas la
// voie géométrique (0,15 m) : sinon l'odométrie surestime les virages d'environ 50 %.
[[nodiscard]] Twist wheelsToTwist(const WheelRates& wheels, double radius, double track) noexcept;

// (v, ω) → roues : la cinématique inverse, pour transformer une commande en consignes moteur.
[[nodiscard]] WheelRates twistToWheels(const Twist& twist, double radius, double track) noexcept;

} // namespace fieldnav::odometry
