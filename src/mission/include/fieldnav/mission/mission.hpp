// Une mission complète dans la simulation : le robot suit le rang en boucle fermée.
//   sim MuJoCo → roues bruitées → odométrie ─┐
//   sim MuJoCo → GPS bruité ─────────────────┴→ fusion → pose estimée → pure pursuit → roues
// Utilisé par apps/fieldnav_run (ligne de commande) et par les tests d'intégration.
#pragma once

#include <cstddef>
#include <cstdint>
#include <fieldnav/sensors/gps_sim.hpp>
#include <optional>
#include <string>

namespace fieldnav::mission {

// Pose donnée au contrôleur : la vérité terrain (référence), l'odométrie seule, ou la fusion.
enum class PoseSource { Truth, Odometry, Fusion };

[[nodiscard]] std::optional<PoseSource>
parsePoseSource(const std::string& name); // truth|odom|fusion
[[nodiscard]] const char* toString(PoseSource source);

struct MissionConfig {
    std::string scene_file;     // scenes/row_<nom>.xml
    std::string reference_file; // data/reference_<nom>.csv
    PoseSource pose_source{PoseSource::Fusion};
    std::uint32_t seed{42};
    double max_duration{300.0}; // s ; la mission s'arrête avant si le robot atteint le bout du rang
    sensors::GpsParams gps{};   // σ 1 m, 20 % de pertes, 5 Hz ; trou GNSS désactivé par défaut
    double lookahead{0.8};      // m, pure pursuit
    double v_des{0.3};          // m/s
    double alpha{0.05};         // gain de la fusion complémentaire
    double off_row_limit{0.75}; // m : |erreur latérale vraie| au-delà = sortie de rang (code 1)
    std::string telemetry_file; // vide = pas de télémétrie ; .csv ou .jsonl
};

struct MissionResult {
    double duration{0.0};     // s
    double distance{0.0};     // m parcourus le long du rang
    double rms_lateral{0.0};  // m, erreur latérale VRAIE, RMS sur toute la mission
    double max_lateral{0.0};  // m, |erreur latérale| maximale
    double rms_estimate{0.0}; // m, erreur de position de l'estimée (RMS)
    bool reached_end{false};  // le robot est arrivé au bout du rang
    bool off_row{false};      // arrêt pour sortie de rang
    std::size_t telemetry_samples{0};

    // Code retour de fieldnav_run : 0 ok, 1 sortie de rang (2 = erreur de config, géré avant).
    [[nodiscard]] int exitCode() const noexcept { return off_row ? 1 : 0; }
};

// Lance la mission. Lance std::runtime_error si la scène ou la référence ne se chargent pas.
[[nodiscard]] MissionResult runMission(const MissionConfig& config);

} // namespace fieldnav::mission
