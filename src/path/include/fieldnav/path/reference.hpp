// Trajectoire de référence d'un rang : la ligne médiane, lue depuis data/reference_<nom>.csv
// (généré par scripts/gen_row_scene.py). C'est elle que Path et le pure pursuit suivront.
// Rien à écrire dans ce fichier : le code va dans src/path/reference.cpp.
#pragma once

#include <string>
#include <vector>

namespace fieldnav::path {

// Un point de la ligne médiane : abscisse curviligne s (m, distance le long du rang) et position.
struct RefPoint {
    double s{0.0};
    double x{0.0};
    double y{0.0};
};

// Lit un CSV « s,x,y » avec une ligne d'en-tête. Lance std::runtime_error si le fichier est
// introuvable ou si une ligne n'a pas exactement 3 nombres (le message donne le numéro de ligne).
[[nodiscard]] std::vector<RefPoint> loadReferenceCsv(const std::string& file_path);

} // namespace fieldnav::path
