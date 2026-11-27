// fieldnav_run : lance une mission de suivi de rang dans la simulation MuJoCo et écrit la
// télémétrie. Exemple :
//   fieldnav_run --scene row_straight --pose-source fusion --seed 42
//                --gps-outage 80,90 --out results/run.csv
// Codes retour : 0 ok, 1 sortie de rang, 2 erreur de configuration (option invalide, fichier
// introuvable).
//
// Décisions non évidentes :
// 1. Pas de bibliothèque d'options (CLI11, cxxopts) : CLAUDE.md interdit les nouvelles
//    dépendances, et une douzaine d'options « --nom valeur » se lisent en 60 lignes.
// 2. --scene accepte un nom (row_straight) ou un chemin (.xml). Avec un nom, la scène est
//    cherchée dans scenes/ et la référence déduite : row_<x>.xml → data/reference_<x>.csv.
//    Les deux dossiers sont fixés à la compilation (FIELDNAV_SCENES_DIR, FIELDNAV_DATA_DIR) :
//    le programme marche depuis n'importe quel dossier courant, y compris build/.
// 3. Toute erreur d'option ou de fichier sort avec le code 2 AVANT de lancer la simulation :
//    run_matrix.sh et la CI distinguent ainsi « mauvaise configuration » (2) de « le robot est
//    sorti du rang » (1).
#include <cstdio>
#include <cstdlib>
#include <exception>
#include <fieldnav/mission/mission.hpp>
#include <stdexcept>
#include <string>

#ifdef _WIN32
#include <windows.h>
#endif

namespace {

namespace fm = fieldnav::mission;

constexpr int kExitConfig = 2;

void usage() {
    std::printf("Usage : fieldnav_run [options]\n"
                "  --scene NOM|FICHIER.xml   scène (défaut : row_straight)\n"
                "  --reference FICHIER.csv   référence (défaut : déduite de la scène)\n"
                "  --pose-source SOURCE      truth | odom | fusion (défaut : fusion)\n"
                "  --seed N                  graine de l'aléa (défaut : 42)\n"
                "  --duration S              durée maximale en s (défaut : 300)\n"
                "  --gps-sigma M             bruit GPS en m (défaut : 1.0)\n"
                "  --gps-loss P              taux de pertes GPS, 0 à 1 (défaut : 0.2)\n"
                "  --gps-outage T1,T2        trou GNSS entre T1 et T2 s (défaut : aucun)\n"
                "  --lookahead M             distance visée du pure pursuit (défaut : 0.8)\n"
                "  --alpha A                 gain de la fusion (défaut : 0.05)\n"
                "  --out FICHIER             télémétrie .csv ou .jsonl (défaut : aucune)\n"
                "Codes retour : 0 ok, 1 sortie de rang, 2 erreur de configuration.\n");
}

double toNumber(const std::string& option, const std::string& text) {
    std::size_t used = 0;
    double value = 0.0;
    try {
        value = std::stod(text, &used);
    } catch (const std::exception&) {
        used = 0;
    }
    if (used == 0 || used != text.size()) {
        throw std::invalid_argument(option + " : nombre attendu, reçu « " + text + " »");
    }
    return value;
}

bool endsWith(const std::string& s, const std::string& suffix) {
    return s.size() >= suffix.size() &&
           s.compare(s.size() - suffix.size(), suffix.size(), suffix) == 0;
}

fm::MissionConfig parse(int argc, char** argv) {
    fm::MissionConfig cfg;
    std::string scene = "row_straight";
    for (int i = 1; i < argc; ++i) {
        const std::string opt = argv[i];
        if (opt == "--help" || opt == "-h") {
            usage();
            std::exit(0);
        }
        if (i + 1 >= argc) {
            throw std::invalid_argument(opt + " : valeur manquante");
        }
        const std::string val = argv[++i];
        if (opt == "--scene") {
            scene = val;
        } else if (opt == "--reference") {
            cfg.reference_file = val;
        } else if (opt == "--pose-source") {
            const auto source = fm::parsePoseSource(val);
            if (!source) {
                throw std::invalid_argument("--pose-source : truth, odom ou fusion, reçu « " + val +
                                            " »");
            }
            cfg.pose_source = *source;
        } else if (opt == "--seed") {
            cfg.seed = static_cast<std::uint32_t>(toNumber(opt, val));
        } else if (opt == "--duration") {
            cfg.max_duration = toNumber(opt, val);
        } else if (opt == "--gps-sigma") {
            cfg.gps.sigma_noise = toNumber(opt, val);
        } else if (opt == "--gps-loss") {
            cfg.gps.p_loss = toNumber(opt, val);
        } else if (opt == "--gps-outage") {
            const auto comma = val.find(',');
            if (comma == std::string::npos) {
                throw std::invalid_argument("--gps-outage : format T1,T2 attendu");
            }
            cfg.gps.outage_start = toNumber(opt, val.substr(0, comma));
            cfg.gps.outage_end = toNumber(opt, val.substr(comma + 1));
        } else if (opt == "--lookahead") {
            cfg.lookahead = toNumber(opt, val);
        } else if (opt == "--alpha") {
            cfg.alpha = toNumber(opt, val);
        } else if (opt == "--out") {
            cfg.telemetry_file = val;
        } else {
            throw std::invalid_argument("option inconnue : " + opt);
        }
    }

    // Scène : un nom (cherché dans scenes/) ou un chemin vers un .xml.
    if (endsWith(scene, ".xml")) {
        cfg.scene_file = scene;
    } else {
        cfg.scene_file = std::string(FIELDNAV_SCENES_DIR) + "/" + scene + ".xml";
    }
    if (cfg.reference_file.empty()) {
        // row_<x> → reference_<x>.csv
        std::string base = scene.substr(scene.find_last_of("/\\") + 1);
        if (endsWith(base, ".xml")) {
            base.resize(base.size() - 4);
        }
        if (base.rfind("row_", 0) != 0) {
            throw std::invalid_argument("--reference requis : la scène « " + scene +
                                        " » ne suit pas le nommage row_<nom>");
        }
        cfg.reference_file =
            std::string(FIELDNAV_DATA_DIR) + "/reference_" + base.substr(4) + ".csv";
    }
    if (cfg.gps.p_loss < 0.0 || cfg.gps.p_loss > 1.0 || cfg.gps.sigma_noise < 0.0 ||
        cfg.lookahead <= 0.0 || cfg.alpha < 0.0 || cfg.alpha > 1.0 || cfg.max_duration <= 0.0) {
        throw std::invalid_argument("valeur hors limites (voir --help)");
    }
    return cfg;
}

} // namespace

int main(int argc, char** argv) {
#ifdef _WIN32
    // La console Windows attend par défaut une ancienne page de code (850) : sans cela, les
    // accents des messages UTF-8 s'affichent « t├®l├®m├®trie ».
    SetConsoleOutputCP(CP_UTF8);
#endif
    fm::MissionConfig cfg;
    try {
        cfg = parse(argc, argv);
    } catch (const std::exception& e) {
        std::fprintf(stderr, "fieldnav_run : %s\n", e.what());
        return kExitConfig;
    }

    fm::MissionResult r;
    try {
        r = fm::runMission(cfg);
    } catch (const std::exception& e) {
        std::fprintf(stderr, "fieldnav_run : %s\n", e.what());
        return kExitConfig;
    }

    std::printf("FieldNav — scène %s, pose %s, seed %u\n", cfg.scene_file.c_str(),
                fm::toString(cfg.pose_source), cfg.seed);
    std::printf("  distance parcourue  : %.2f m en %.1f s%s\n", r.distance, r.duration,
                r.reached_end ? " (bout du rang atteint)" : "");
    std::printf("  erreur latérale RMS : %.3f m (max %.3f m)\n", r.rms_lateral, r.max_lateral);
    std::printf("  erreur de l'estimée : %.3f m (RMS de position)\n", r.rms_estimate);
    if (!cfg.telemetry_file.empty()) {
        std::printf("  télémétrie          : %s (%zu lignes)\n", cfg.telemetry_file.c_str(),
                    r.telemetry_samples);
    }
    if (r.off_row) {
        std::printf("  SORTIE DE RANG (|erreur latérale| > %.2f m)\n", cfg.off_row_limit);
    }
    return r.exitCode();
}
