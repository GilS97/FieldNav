// Lecture du CSV de trajectoire de référence (s,x,y).

#include <fieldnav/path/reference.hpp>
#include <fstream>
#include <sstream>
#include <stdexcept>

namespace fieldnav::path {

namespace {

// Découpe une ligne « s,x,y » ; refuse tout ce qui n'est pas exactement trois nombres.
// Tolère une fin de ligne Windows : std::getline coupe sur '\n' et laisse le '\r' de « \r\n ».
RefPoint parseLine(std::string line, int line_number) {
    if (!line.empty() && line.back() == '\r') {
        line.pop_back();
    }
    std::istringstream fields(line);
    std::string field;
    double values[3] = {0.0, 0.0, 0.0};
    int count = 0;
    while (std::getline(fields, field, ',')) {
        if (count == 3) {
            count = 4; // trop de colonnes
            break;
        }
        std::size_t used = 0;
        try {
            values[count] = std::stod(field, &used);
        } catch (const std::exception&) {
            used = 0;
        }
        if (used == 0 || used != field.size()) {
            throw std::runtime_error("ligne " + std::to_string(line_number) +
                                     " : nombre invalide « " + field + " »");
        }
        ++count;
    }
    if (count != 3) {
        throw std::runtime_error("ligne " + std::to_string(line_number) +
                                 " : 3 colonnes attendues (s,x,y)");
    }
    return RefPoint{values[0], values[1], values[2]};
}

} // namespace

std::vector<RefPoint> loadReferenceCsv(const std::string& file_path) {
    std::ifstream in(file_path);
    if (!in) {
        throw std::runtime_error("référence introuvable : " + file_path);
    }
    std::string line;
    std::getline(in, line); // en-tête « s,x,y », ignoré

    std::vector<RefPoint> points;
    int line_number = 1;
    while (std::getline(in, line)) {
        ++line_number;
        if (line.empty())
            continue;
        points.push_back(parseLine(line, line_number));
    }
    return points;
}

} // namespace fieldnav::path
