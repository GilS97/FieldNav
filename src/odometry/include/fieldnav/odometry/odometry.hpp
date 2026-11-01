// Odométrie d'un robot différentiel : intègre les vitesses (v, ω) pour estimer la pose.
// Repère : départ du rang = origine, x vers l'avant, angles en radians, temps en secondes.
#pragma once

namespace fieldnav::odometry {

// Pose 2D du robot dans le repère du rang.
struct Pose2D {
    double x{0.0};     // m, vers l'avant le long du rang
    double y{0.0};     // m, vers la gauche
    double theta{0.0}; // rad, dans [-π, π], 0 = aligné sur le rang
};

// Ramène un angle quelconque (rad) dans [-π, π].
// Exemples : 3π/2 → -π/2 ; -3π/2 → π/2 ; 10π + 0,1 → 0,1.
[[nodiscard]] double normalizeAngle(double angle) noexcept;

// Intègre (v, ω) pas à pas. Aucun bruit ici : le bruit des roues est ajouté par la sim.
class DiffDriveOdometry {
public:
    DiffDriveOdometry() = default;
    explicit DiffDriveOdometry(const Pose2D& initial) noexcept : pose_(initial) {}

    // Un pas d'Euler : v en m/s, w (ω) en rad/s, dt en s (dt > 0).
    void update(double v, double w, double dt) noexcept;

    [[nodiscard]] const Pose2D& pose() const noexcept { return pose_; }
    void reset(const Pose2D& pose = {}) noexcept { pose_ = pose; }

private:
    Pose2D pose_{};
};

} // namespace fieldnav::odometry
