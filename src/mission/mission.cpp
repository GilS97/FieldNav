// Boucle de mission : sim → capteurs → estimée → pure pursuit → roues, et télémétrie.
//
// 1. Trois fréquences. La physique tourne à 500 Hz (pas MuJoCo de 2 ms), le contrôleur à
//    50 Hz et la télémétrie à 10 Hz. Le vrai robot (ESP8266, tâche du 12 déc.) bouclera à 50 Hz
//    et publiera en MQTT à 10 Hz : la simulation commande et enregistre au même rythme que lui.
//    L'odométrie, la fusion et l'erreur RMS, elles, sont mises à jour à chaque pas de 2 ms.
// 2. Deux entraxes. L'odométrie convertit les roues MESURÉES avec l'entraxe effectif 0,22 m ;
//    le contrôleur convertit (v, ω) en CONSIGNES avec 0,36 m. Les deux ont été mesurés en sim
//    (2 nov.) : en virage les moteurs saturent, les roues n'atteignent pas leur consigne.
// 3. Critère d'arrêt. Le robot s'arrête quand sa projection VRAIE sur le rang atteint la fin
//    (à 5 cm près) : en bout de rang le point visé est le dernier point, et sans ce critère le
//    pure pursuit ferait demi-tour après l'avoir dépassé (le demi-tour est hors périmètre).
//    Il s'arrête aussi si |erreur latérale vraie| > off_row_limit (0,75 m, la demi-largeur du
//    rang : le robot écraserait les plants).
// 4. Chemin prolongé pour le contrôleur. Près du bout du rang, le point visé resterait collé
//    au dernier point : la distance L_d tend vers 0 et la courbure κ = 2 sin α / L_d explose
//    (oscillation de ±40 cm dans les 4 dernières secondes, vue sur les figures du 27 nov.).
//    Le contrôleur suit donc le rang prolongé en ligne droite de lookahead + 1 m ; l'erreur
//    latérale et la fin de mission, elles, se mesurent sur le vrai rang.
#include <cmath>
#include <fieldnav/control/pure_pursuit.hpp>
#include <fieldnav/estimation/complementary.hpp>
#include <fieldnav/mission/mission.hpp>
#include <fieldnav/odometry/kinematics.hpp>
#include <fieldnav/odometry/odometry.hpp>
#include <fieldnav/path/path.hpp>
#include <fieldnav/sensors/wheel_noise.hpp>
#include <fieldnav/sim/diff_drive_sim.hpp>
#include <fieldnav/telemetry/telemetry_writer.hpp>
#include <memory>
#include <utility>
#include <vector>

namespace fieldnav::mission {

namespace {

constexpr double kTrackOdometry = 0.22; // m, entraxe effectif (roues mesurées)
constexpr double kTrackCommand = 0.36;  // m, entraxe de commande (consignes)
constexpr int kControlDivider = 10;     // 500 Hz / 10 = 50 Hz
constexpr int kTelemetryDivider = 50;   // 500 Hz / 50 = 10 Hz
constexpr double kEndTolerance = 0.05;  // m

// Le rang, prolongé tout droit (direction du dernier segment) de `extra` mètres, un point tous
// les 10 cm : le point visé par le pure pursuit reste à L_d ≈ lookahead jusqu'au bout.
path::Path extendedPath(std::vector<path::RefPoint> pts, double extra) {
    const path::RefPoint a = pts[pts.size() - 2];
    const path::RefPoint b = pts.back();
    const double len = std::hypot(b.x - a.x, b.y - a.y);
    const double tx = (b.x - a.x) / len;
    const double ty = (b.y - a.y) / len;
    const auto n = static_cast<int>(std::ceil(extra / 0.1));
    for (int k = 1; k <= n; ++k) {
        const double d = 0.1 * k;
        pts.push_back({b.s + d, b.x + tx * d, b.y + ty * d});
    }
    return path::Path(std::move(pts));
}

} // namespace

std::optional<PoseSource> parsePoseSource(const std::string& name) {
    if (name == "truth") {
        return PoseSource::Truth;
    }
    if (name == "odom") {
        return PoseSource::Odometry;
    }
    if (name == "fusion") {
        return PoseSource::Fusion;
    }
    return std::nullopt;
}

const char* toString(PoseSource source) {
    switch (source) {
    case PoseSource::Truth:
        return "truth";
    case PoseSource::Odometry:
        return "odom";
    case PoseSource::Fusion:
        return "fusion";
    }
    return "?";
}

MissionResult runMission(const MissionConfig& cfg) {
    sim::DiffDriveSim sim(cfg.scene_file);
    const auto reference = path::loadReferenceCsv(cfg.reference_file);
    path::Path truth_path(reference); // le vrai rang : erreur latérale et fin de mission
    path::Path controller_path = extendedPath(reference, cfg.lookahead + 1.0); // décision 4

    // Seeds dérivées de la seed du run : chaque source d'aléa a son générateur.
    sensors::WheelNoise noise_left(cfg.seed);
    sensors::WheelNoise noise_right(cfg.seed + 1);
    sensors::GpsSim gps(cfg.seed + 2, cfg.gps);
    odometry::DiffDriveOdometry odom;
    estimation::ComplementaryFuser fusion(cfg.alpha);

    control::PurePursuitParams pp;
    pp.lookahead = cfg.lookahead;
    pp.v_des = cfg.v_des;

    std::unique_ptr<telemetry::TelemetryWriter> writer;
    if (!cfg.telemetry_file.empty()) {
        writer = std::make_unique<telemetry::TelemetryWriter>(
            cfg.telemetry_file, telemetry::TelemetryWriter::formatFromPath(cfg.telemetry_file));
    }

    const double dt = sim.timestep();
    const auto max_steps = static_cast<long>(std::lround(cfg.max_duration / dt));
    control::Cmd cmd{};
    std::optional<sensors::GpsFix> fix_since_log; // dernière mesure GPS depuis la ligne précédente
    double sum_lat2 = 0.0, sum_est2 = 0.0;
    long n = 0;
    MissionResult result;

    for (long i = 0; i < max_steps; ++i) {
        // Contrôleur (50 Hz), à partir de la pose choisie.
        if (i % kControlDivider == 0) {
            const odometry::Pose2D est = cfg.pose_source == PoseSource::Truth ? sim.truePose()
                                         : cfg.pose_source == PoseSource::Odometry
                                             ? odom.pose()
                                             : fusion.estimate();
            cmd = control::purePursuit(est, controller_path, pp);
            const auto wheels =
                odometry::twistToWheels({cmd.v, cmd.w}, sim.wheelRadius(), kTrackCommand);
            sim.setWheelSpeeds({wheels.left, wheels.right});
        }

        sim.step();
        const double t = sim.time();

        // Capteurs et estimation (500 Hz).
        const auto measured = sim.wheelSpeeds();
        const auto twist = odometry::wheelsToTwist(
            {noise_left.apply(measured.left), noise_right.apply(measured.right)}, sim.wheelRadius(),
            kTrackOdometry);
        odom.update(twist.v, twist.w, dt);
        fusion.predict(twist.v, twist.w, dt);
        const auto truth = sim.truePose();
        if (const auto fix = gps.update(t, truth.x, truth.y)) {
            fusion.correct(*fix);
            fix_since_log = fix;
        }

        // Mesures de performance sur la vérité.
        const odometry::Pose2D est = cfg.pose_source == PoseSource::Odometry ? odom.pose()
                                     : cfg.pose_source == PoseSource::Fusion ? fusion.estimate()
                                                                             : truth;
        const double lat = truth_path.lateralError(truth);
        sum_lat2 += lat * lat;
        sum_est2 += std::pow(est.x - truth.x, 2) + std::pow(est.y - truth.y, 2);
        result.max_lateral = std::fmax(result.max_lateral, std::fabs(lat));
        ++n;

        // Télémétrie (10 Hz).
        if (writer && (i + 1) % kTelemetryDivider == 0) {
            telemetry::TelemetrySample s;
            s.t_s = t;
            s.x_true = truth.x;
            s.y_true = truth.y;
            s.th_true = truth.theta;
            s.x_est = est.x;
            s.y_est = est.y;
            s.th_est = est.theta;
            if (fix_since_log) {
                s.x_gps = fix_since_log->x;
                s.y_gps = fix_since_log->y;
            }
            s.v_cmd = cmd.v;
            s.w_cmd = cmd.w;
            s.lat_err = lat;
            writer->write(s);
            fix_since_log.reset();
        }

        result.distance = truth_path.arcLength(truth);
        result.duration = t;
        if (std::fabs(lat) > cfg.off_row_limit) {
            result.off_row = true;
            break;
        }
        if (result.distance >= truth_path.length() - kEndTolerance) {
            result.reached_end = true;
            break;
        }
    }

    const double count = static_cast<double>(n > 0 ? n : 1);
    result.rms_lateral = std::sqrt(sum_lat2 / count);
    result.rms_estimate = std::sqrt(sum_est2 / count);
    if (writer) {
        result.telemetry_samples = writer->samples();
        writer->close();
    }
    return result;
}

} // namespace fieldnav::mission
