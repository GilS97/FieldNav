// Le chemin à suivre (la ligne médiane du rang) et les deux mesures dont le contrôleur a besoin :
// l'erreur latérale (à quelle distance de la ligne est le robot, et de quel côté) et le point
// visé à une distance donnée devant lui (lookahead). Rien à écrire dans ce fichier : le code va
// dans src/path/path.cpp.
#pragma once

#include <cstddef>
#include <fieldnav/odometry/odometry.hpp>
#include <fieldnav/path/reference.hpp>
#include <string>
#include <vector>

namespace fieldnav::path {

struct Point2D {
    double x{0.0};
    double y{0.0};
};

class Path {
public:
    // Au moins 2 points, s strictement croissant (lance std::invalid_argument sinon).
    explicit Path(std::vector<RefPoint> points);

    // Charge data/reference_<nom>.csv (voir loadReferenceCsv).
    [[nodiscard]] static Path fromCsv(const std::string& file_path);

    // Indice du point du chemin le plus proche du robot. Recherche LOCALE : on part du dernier
    // indice trouvé et on avance (ou recule) tant que la distance diminue. Coût O(1) amorti à
    // chaque pas, au lieu de parcourir les 501 points ; c'est pour ça que la méthode n'est pas
    // const (elle mémorise l'indice). Le premier appel fait une recherche complète.
    [[nodiscard]] std::size_t closestIndex(const odometry::Pose2D& pose);

    // Erreur latérale SIGNÉE (m) : distance du robot à la ligne, positive si le robot est à
    // GAUCHE de la ligne (dans le sens de parcours), négative à droite. Ne dépend pas du cap.
    [[nodiscard]] double lateralError(const odometry::Pose2D& pose);

    // Point du chemin situé `distance` mètres plus loin que la projection du robot, mesuré le
    // long du chemin (abscisse curviligne), interpolé entre deux points. Au-delà de la fin du
    // chemin : le dernier point.
    [[nodiscard]] Point2D lookahead(const odometry::Pose2D& pose, double distance);

    // Abscisse curviligne de la projection du robot sur le chemin (m) : la distance parcourue le
    // long du rang. Sert à savoir quand le robot est arrivé au bout.
    [[nodiscard]] double arcLength(const odometry::Pose2D& pose) { return project(pose).s; }

    [[nodiscard]] double length() const noexcept { return points_.back().s; }
    [[nodiscard]] const std::vector<RefPoint>& points() const noexcept { return points_; }

private:
    // Segment [i, i + 1] le plus proche du robot (i = closestIndex, ou l'avant-dernier point si
    // closestIndex est le dernier), et abscisse curviligne de la projection du robot sur ce
    // segment. Outils pour lateralError() et lookahead().
    struct Projection {
        std::size_t segment{0}; // indice i du segment [i, i + 1]
        double s{0.0};          // abscisse curviligne de la projection (m)
    };
    [[nodiscard]] Projection project(const odometry::Pose2D& pose);

    std::vector<RefPoint> points_;
    std::size_t last_index_{0};
    bool has_last_index_{false};
};

} // namespace fieldnav::path
