// GPS simulé : position vraie + biais lent (marche aléatoire) + bruit blanc, à 5 Hz, avec des
// mesures perdues et un trou GNSS optionnel. Rien à écrire dans ce fichier : le code va dans
// src/sensors/gps_sim.cpp.
//
// Reproductible (CLAUDE.md) : tout l'aléa vient d'un std::mt19937 seedé, interne à l'objet, et
// le même nombre de tirages est fait à chaque instant d'échantillonnage, qu'il y ait une mesure
// ou non (perte, trou GNSS). Même seed = même suite de mesures.
#pragma once

#include <cstdint>
#include <optional>
#include <random>

namespace fieldnav::sensors {

// Une mesure GPS : instant (s) et position (m) dans le repère du rang.
struct GpsFix {
    double t{0.0};
    double x{0.0};
    double y{0.0};
};

struct GpsParams {
    double sigma_noise{1.0}; // m, écart-type du bruit blanc de chaque mesure
    double sigma_rw{0.02};   // m/√s, intensité de la marche aléatoire du biais
    double p_loss{0.2};      // probabilité qu'une mesure soit perdue
    double rate_hz{5.0};     // fréquence des mesures
    // Trou GNSS : aucune mesure pour t dans [outage_start, outage_end]. Négatif = pas de trou.
    double outage_start{-1.0};
    double outage_end{-1.0};
};

class GpsSim {
public:
    explicit GpsSim(std::uint32_t seed, const GpsParams& params = {});

    // Appelée à chaque pas de simulation avec le temps et la position vraie du robot.
    // Renvoie une mesure seulement aux instants d'échantillonnage (toutes les 1/rate_hz s),
    // et std::nullopt sinon, si la mesure est perdue, ou pendant le trou GNSS.
    [[nodiscard]] std::optional<GpsFix> update(double t, double true_x, double true_y);

    // Biais courant (m), exposé pour les tests.
    [[nodiscard]] double biasX() const noexcept { return bias_x_; }
    [[nodiscard]] double biasY() const noexcept { return bias_y_; }

private:
    // Les trois étapes du modèle, appelées par update() dans cet ordre à chaque échantillon.
    void stepBias(double dt);                                   // marche aléatoire du biais
    [[nodiscard]] GpsFix measure(double t, double x, double y); // vrai + biais + bruit
    [[nodiscard]] bool isLost();                                // tirage de perte

    GpsParams p_;
    std::mt19937 rng_;
    std::normal_distribution<double> unit_normal_;        // N(0, 1)
    std::uniform_real_distribution<double> unit_uniform_; // U[0, 1)
    double bias_x_{0.0};
    double bias_y_{0.0};
    long sample_index_{0}; // échantillons déjà faits (prochain instant = (index + 1) / rate_hz)
};

} // namespace fieldnav::sensors
