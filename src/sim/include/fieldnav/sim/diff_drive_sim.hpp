// Simulation physique MuJoCo du robot (scenes/robot_diffdrive.xml : LAFVIN 4WD, skid-steer).
// Donne la pose VRAIE (vérité terrain) et les vitesses de roues MESURÉES par côté, sans bruit :
// le bruit ±2 % et l'odométrie sont ajoutés par-dessus, dans la boucle de run.
#pragma once

#include <fieldnav/odometry/odometry.hpp>
#include <string>

// Types MuJoCo déclarés sans inclure <mujoco/mujoco.h> : seul diff_drive_sim.cpp en dépend.
struct mjModel_;
struct mjData_;

namespace fieldnav::sim {

// Vitesses angulaires des roues de chaque côté, en rad/s (positif = fait avancer le robot).
struct WheelSpeeds {
    double left{0.0};
    double right{0.0};
};

class DiffDriveSim {
public:
    // Charge le modèle MJCF. Lance std::runtime_error si le fichier est absent ou invalide.
    explicit DiffDriveSim(const std::string& model_path);
    ~DiffDriveSim();

    DiffDriveSim(const DiffDriveSim&) = delete;
    DiffDriveSim& operator=(const DiffDriveSim&) = delete;
    DiffDriveSim(DiffDriveSim&&) = delete;
    DiffDriveSim& operator=(DiffDriveSim&&) = delete;

    // Consigne de vitesse des roues (rad/s), appliquée aux pas suivants.
    void setWheelSpeeds(const WheelSpeeds& target) noexcept;

    // Avance la physique d'un pas de timestep() secondes.
    void step() noexcept;

    // Remet le robot à l'origine, immobile, temps à 0.
    void reset() noexcept;

    // Pose vraie du châssis dans le plan : x, y (m) et cap theta (rad, dans [-π, π]).
    [[nodiscard]] odometry::Pose2D truePose() const noexcept;

    // Vitesses de roues réellement atteintes (rad/s), lues dans la physique.
    [[nodiscard]] WheelSpeeds wheelSpeeds() const noexcept;

    [[nodiscard]] double time() const noexcept;
    [[nodiscard]] double timestep() const noexcept;
    [[nodiscard]] double wheelRadius() const noexcept { return wheel_radius_; }
    [[nodiscard]] double trackWidth() const noexcept { return track_width_; }

private:
    mjModel_* model_{nullptr};
    mjData_* data_{nullptr};
    int qpos_root_{-1}; // index de début de la freejoint dans data->qpos (x y z qw qx qy qz)
    // Index des 4 roues dans data->qvel (fl = avant gauche, rl = arrière gauche, ...).
    int dof_fl_{-1};
    int dof_rl_{-1};
    int dof_fr_{-1};
    int dof_rr_{-1};
    // Index des 4 moteurs dans data->ctrl.
    int act_fl_{-1};
    int act_rl_{-1};
    int act_fr_{-1};
    int act_rr_{-1};
    double wheel_radius_{0.0}; // m, lu dans le modèle
    double track_width_{0.0};  // m, voie (centre pneu gauche - pneu droit), lue dans le modèle
};

} // namespace fieldnav::sim
