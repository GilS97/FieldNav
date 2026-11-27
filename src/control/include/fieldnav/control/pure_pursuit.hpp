// Pure pursuit : à chaque pas, viser un point du chemin situé L_d mètres devant le robot et
// calculer l'arc de cercle qui y mène. C'est le seul contrôleur de FieldNav jusqu'en janvier.
// Rien à écrire dans ce fichier : le code va dans src/control/pure_pursuit.cpp.
#pragma once

#include <fieldnav/odometry/odometry.hpp>
#include <fieldnav/path/path.hpp>

namespace fieldnav::control {

// Commande du robot : v (m/s) le long du cap, w = ω (rad/s), positif = vers la gauche.
struct Cmd {
    double v{0.0};
    double w{0.0};
};

struct PurePursuitParams {
    // Distance du point visé (m) : LE réglage du pure pursuit. Petit = réactif mais oscille ;
    // grand = lisse mais coupe les virages. Valeur retenue à noter dans le journal (CLAUDE.md).
    double lookahead{0.8};
    double v_des{0.3}; // vitesse d'avance voulue (m/s), < 0,64 m/s (moteurs TT à vide)
    double w_max{1.5}; // saturation de ω (rad/s)
};

// Commande pour suivre `path` depuis `pose`. Non-const sur path : Path mémorise l'indice le plus
// proche (recherche locale).
[[nodiscard]] Cmd purePursuit(const odometry::Pose2D& pose, path::Path& path,
                              const PurePursuitParams& params = {});

} // namespace fieldnav::control
