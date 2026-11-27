// Écriture CSV / JSON Lines de la télémétrie (docs/telemetry_format.md).
//
// Trois décisions non évidentes :
// 1. Fichier ouvert en mode BINAIRE : sous Windows, le mode texte écrirait « \r\n ». En binaire,
//    les fins de ligne restent « \n » partout, et le même run donne le même fichier octet pour
//    octet sous Windows et dans la CI Linux (règle de reproductibilité de CLAUDE.md).
// 2. Nombres écrits avec un nombre FIXE de décimales (snprintf "%.*f"), jamais avec << :
//    l'affichage par défaut des flux dépend de la locale et de la valeur (notation scientifique
//    pour 1e-5), ce qui rendrait le fichier illisible pour un diff et pour plot_run.py.
//    « -0.0000 » est réécrit « 0.0000 » : même valeur, et un diff ne montre pas de faux écart.
// 3. Valeur absente = champ vide en CSV, null en JSONL, et non « nan » : l'écriture de NaN par
//    printf diffère selon la bibliothèque C (« nan », « -nan(ind) »…), pandas lit le champ
//    vide comme NaN, et null est du JSON valide (NaN ne l'est pas).
#include <cstdio>
#include <fieldnav/telemetry/telemetry_writer.hpp>
#include <iterator>
#include <stdexcept>
#include <string>

namespace fieldnav::telemetry {

namespace {

constexpr const char* kColumns[] = {"t_s",   "x_true", "y_true", "th_true", "x_est",
                                    "y_est", "th_est", "x_gps",  "y_gps",   "gps_valid",
                                    "v_cmd", "w_cmd",  "lat_err"};

std::string fixed(double value, int decimals) {
    char buf[64];
    std::snprintf(buf, sizeof(buf), "%.*f", decimals, value);
    std::string s(buf);
    if (s[0] == '-' && s.find_first_not_of("-0.") == std::string::npos) {
        s.erase(0, 1); // « -0.0000 » → « 0.0000 »
    }
    return s;
}

std::string opt(const std::optional<double>& value, int decimals, const char* missing) {
    return value ? fixed(*value, decimals) : std::string(missing);
}

} // namespace

TelemetryWriter::TelemetryWriter(const std::string& file_path, TelemetryFormat format)
    : out_(file_path, std::ios::binary | std::ios::trunc), format_(format) {
    if (!out_) {
        throw std::runtime_error("télémétrie : impossible de créer " + file_path);
    }
    if (format_ == TelemetryFormat::Csv) {
        std::string header;
        for (const char* c : kColumns) {
            header += (header.empty() ? "" : ",") + std::string(c);
        }
        out_ << header << '\n';
    }
}

TelemetryWriter::~TelemetryWriter() {
    close();
}

TelemetryFormat TelemetryWriter::formatFromPath(const std::string& file_path) {
    const std::string ext = ".jsonl";
    const bool jsonl = file_path.size() >= ext.size() &&
                       file_path.compare(file_path.size() - ext.size(), ext.size(), ext) == 0;
    return jsonl ? TelemetryFormat::JsonLines : TelemetryFormat::Csv;
}

void TelemetryWriter::write(const TelemetrySample& s) {
    const bool csv = format_ == TelemetryFormat::Csv;
    const char* missing = csv ? "" : "null";
    const std::string values[] = {fixed(s.t_s, 3),           opt(s.x_true, 4, missing),
                                  opt(s.y_true, 4, missing), opt(s.th_true, 5, missing),
                                  fixed(s.x_est, 4),         fixed(s.y_est, 4),
                                  fixed(s.th_est, 5),        opt(s.x_gps, 4, missing),
                                  opt(s.y_gps, 4, missing),  s.x_gps ? "1" : "0",
                                  fixed(s.v_cmd, 4),         fixed(s.w_cmd, 4),
                                  opt(s.lat_err, 4, missing)};
    std::string line = csv ? "" : "{";
    for (std::size_t i = 0; i < std::size(values); ++i) {
        if (i > 0) {
            line += ',';
        }
        if (!csv) {
            line += std::string("\"") + kColumns[i] + "\":";
        }
        line += values[i];
    }
    if (!csv) {
        line += '}';
    }
    out_ << line << '\n';
    // Vidage régulier : si le programme est interrompu, le fichier reste lisible jusqu'à la
    // dernière seconde (10 lignes à 10 Hz).
    if (++count_ % 10 == 0) {
        out_.flush();
    }
}

void TelemetryWriter::close() {
    if (out_.is_open()) {
        out_.flush();
        out_.close();
    }
}

} // namespace fieldnav::telemetry
