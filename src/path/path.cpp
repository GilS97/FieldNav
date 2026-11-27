// Chemin à suivre : recherche du point le plus proche, erreur latérale, point visé (lookahead).

#include <algorithm>
#include <cmath>
#include <fieldnav/path/path.hpp>
#include <stdexcept>
#include <utility>

namespace fieldnav::path {
namespace {
double distance2(const RefPoint& p, const odometry::Pose2D& pose) {
    const double dx = pose.x - p.x;
    const double dy = pose.y - p.y;
    return dx * dx + dy * dy;
}

} // namespace

Path::Path(std::vector<RefPoint> points) : points_(std::move(points)) {
    if (points_.size() < 2) {
        throw std::invalid_argument("Path : il faut au moins 2 points");
    }
    for (std::size_t i = 1; i < points_.size(); ++i) {
        if (!(points_[i].s > points_[i - 1].s)) {
            throw std::invalid_argument("Path : s doit être strictement croissant (indice " +
                                        std::to_string(i) + ")");
        }
    }
}

Path Path::fromCsv(const std::string& file_path) {
    return Path(loadReferenceCsv(file_path));
}

std::size_t Path::closestIndex(const odometry::Pose2D& pose) {
    if (!has_last_index_) {
        // Premier appel : recherche complète, une seule fois.
        std::size_t best = 0;
        for (std::size_t i = 1; i < points_.size(); ++i) {
            if (distance2(points_[i], pose) < distance2(points_[best], pose)) {
                best = i;
            }
        }
        last_index_ = best;
        has_last_index_ = true;
        return best;
    }
    // Recherche locale : le robot avance de quelques mm par pas, le point le plus proche est
    // donc le même ou un voisin. On descend la pente de la distance dans les deux sens.
    std::size_t i = last_index_;
    while (i + 1 < points_.size() &&
           distance2(points_[i + 1], pose) < distance2(points_[i], pose)) {
        ++i;
    }
    while (i > 0 && distance2(points_[i - 1], pose) < distance2(points_[i], pose)) {
        --i;
    }
    last_index_ = i;
    return i;
}

Path::Projection Path::project(const odometry::Pose2D& pose) {
    const std::size_t i = closestIndex(pose);
    // Le point le plus proche appartient à deux segments, [i−1, i] et [i, i+1] : on garde celui
    // sur lequel la projection tombe (produit scalaire positif depuis le point i).
    std::size_t seg = std::min(i, points_.size() - 2);
    if (i > 0 && i + 1 < points_.size()) {
        const RefPoint& a = points_[i];
        const RefPoint& b = points_[i + 1];
        const double along = (b.x - a.x) * (pose.x - a.x) + (b.y - a.y) * (pose.y - a.y);
        if (along < 0.0) {
            seg = i - 1; // le robot est « avant » le point i : segment précédent
        }
    }
    const RefPoint& a = points_[seg];
    const RefPoint& b = points_[seg + 1];
    const double len = std::hypot(b.x - a.x, b.y - a.y);
    const double tx = (b.x - a.x) / len;
    const double ty = (b.y - a.y) / len;
    // Produit scalaire t · r, borné au segment ; s_projection = s_A + cette distance.
    const double along = std::clamp(tx * (pose.x - a.x) + ty * (pose.y - a.y), 0.0, len);
    return Projection{seg, a.s + along};
}

double Path::lateralError(const odometry::Pose2D& pose) {
    const std::size_t seg = project(pose).segment;
    const RefPoint& a = points_[seg];
    const RefPoint& b = points_[seg + 1];
    const double len = std::hypot(b.x - a.x, b.y - a.y);

    const double tx = (b.x - a.x) / len;
    const double ty = (b.y - a.y) / len;
    const double rx = pose.x - a.x;
    const double ry = pose.y - a.y;
    return tx * ry - ty * rx;
}

Point2D Path::lookahead(const odometry::Pose2D& pose, double distance) {
    const Projection pr = project(pose);
    const double target = pr.s + distance;
    if (target >= length()) {
        return Point2D{points_.back().x, points_.back().y};
    }
    std::size_t j = pr.segment;
    while (points_[j + 1].s < target) { // la boucle ne fait qu'avancer j
        ++j;
    }
    const RefPoint& a = points_[j]; // le segment [j, j+1] contient target
    const RefPoint& b = points_[j + 1];
    const double u = (target - a.s) / (b.s - a.s);
    return Point2D{a.x + u * (b.x - a.x), a.y + u * (b.y - a.y)};
}

} // namespace fieldnav::path
