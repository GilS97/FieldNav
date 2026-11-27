// Écriture de la télémétrie au format de docs/telemetry_format.md (CSV ou JSON Lines).
#pragma once

#include <cstddef>
#include <fstream>
#include <optional>
#include <string>

namespace fieldnav::telemetry {

// Un échantillon : une ligne du fichier. Les champs optionnels absents sont écrits vides (CSV)
// ou null (JSONL) : GPS sans mesure, ou vérité terrain inconnue sur le robot réel.
struct TelemetrySample {
    double t_s{0.0};
    std::optional<double> x_true, y_true, th_true;
    double x_est{0.0}, y_est{0.0}, th_est{0.0};
    std::optional<double> x_gps, y_gps; // gps_valid = x_gps présent
    double v_cmd{0.0}, w_cmd{0.0};
    std::optional<double> lat_err;
};

enum class TelemetryFormat { Csv, JsonLines };

class TelemetryWriter {
public:
    // Ouvre (et écrase) le fichier ; lance std::runtime_error s'il ne peut pas être créé.
    TelemetryWriter(const std::string& file_path, TelemetryFormat format);
    ~TelemetryWriter();

    TelemetryWriter(const TelemetryWriter&) = delete;
    TelemetryWriter& operator=(const TelemetryWriter&) = delete;

    // « run.jsonl » → JSON Lines ; toute autre extension → CSV.
    [[nodiscard]] static TelemetryFormat formatFromPath(const std::string& file_path);

    void write(const TelemetrySample& sample);
    void close();

    [[nodiscard]] std::size_t samples() const noexcept { return count_; }

private:
    std::ofstream out_;
    TelemetryFormat format_;
    std::size_t count_{0};
};

} // namespace fieldnav::telemetry
