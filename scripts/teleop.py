"""Pilotage au clavier du robot FieldNav (LAFVIN 4WD) dans le viewer MuJoCo.

Usage (depuis la racine du dépôt) :
    .venv/Scripts/python scripts/teleop.py            # ouvre le viewer
    .venv/Scripts/python scripts/teleop.py --check    # vérifie la logique sans fenêtre

Touches (choisies pour ne pas entrer en conflit avec celles du viewer) :
    Flèche haut / bas      accélérer / ralentir (pas de 0,05 m/s, marche arrière possible)
    Flèche gauche / droite tourner à gauche / à droite (pas de 0,2 rad/s)
    Suppr                  arrêt (v = 0, ω = 0)
    Backspace              remettre le robot au départ
    Espace                 pause / reprise

Le script convertit la commande (v, ω) en consignes de roues par la cinématique inverse du
robot différentiel, avec un entraxe « de commande » L_CMD mesuré en simulation (skid-steer) :
    ω_gauche = (v − ω·L_CMD/2) / r      ω_droite = (v + ω·L_CMD/2) / r
"""

import argparse
import math
import pathlib
import sys
import time

import mujoco

ROOT = pathlib.Path(__file__).resolve().parents[1]
MODEL = ROOT / "scenes" / "robot_diffdrive.xml"

# Entraxe de commande, mesuré en sim le 2 nov. 2026 pour que ω réel ≈ ω demandé (±20 %).
# Il est plus grand que l'entraxe effectif de l'odométrie (≈ 0,22 m, calculé avec les vitesses
# de roues MESURÉES) parce qu'en virage les moteurs saturent : les roues n'atteignent pas leur
# consigne. Voie géométrique : 0,15 m.
L_CMD = 0.36
V_STEP, W_STEP = 0.05, 0.2  # pas de commande par appui de touche
V_MAX, W_MAX = 0.6, 3.0  # limites de commande (moteurs TT : 20 rad/s ≈ 0,65 m/s)
WHEEL_MAX = 20.0  # rad/s, saturation des moteurs TT

# Codes de touches GLFW.
KEY_UP, KEY_DOWN, KEY_LEFT, KEY_RIGHT, KEY_DELETE = 265, 264, 263, 262, 261
KEY_SPACE, KEY_BACKSPACE = 32, 259


class Robot:
    """Accès au robot dans le modèle MuJoCo (mêmes noms que src/sim/diff_drive_sim.cpp)."""

    def __init__(self, model: mujoco.MjModel, data: mujoco.MjData) -> None:
        self.m, self.d = model, data
        self.radius = model.geom("tire_fl").size[0]
        self.act_left = [model.actuator(n).id for n in ("m_fl", "m_rl")]
        self.act_right = [model.actuator(n).id for n in ("m_fr", "m_rr")]
        self.chassis = model.body("chassis").id
        self.v_cmd = 0.0
        self.w_cmd = 0.0
        self.paused = False
        self.reset_requested = False

    def on_key(self, key: int) -> None:
        # Appelé par la fenêtre du viewer (autre thread) : on ne fait que changer des consignes,
        # la boucle principale les applique sous le verrou.
        if key == KEY_SPACE:
            self.paused = not self.paused
        elif key == KEY_BACKSPACE:
            self.v_cmd = self.w_cmd = 0.0
            self.reset_requested = True
        elif key == KEY_UP:
            self.v_cmd = min(self.v_cmd + V_STEP, V_MAX)
        elif key == KEY_DOWN:
            self.v_cmd = max(self.v_cmd - V_STEP, -V_MAX)
        elif key == KEY_LEFT:
            self.w_cmd = min(self.w_cmd + W_STEP, W_MAX)
        elif key == KEY_RIGHT:
            self.w_cmd = max(self.w_cmd - W_STEP, -W_MAX)
        elif key == KEY_DELETE:
            self.v_cmd = self.w_cmd = 0.0
        # Arrondi pour éviter l'affichage de « -0.00 » après plusieurs appuis.
        self.v_cmd, self.w_cmd = round(self.v_cmd, 3), round(self.w_cmd, 3)

    def apply(self) -> None:
        """Cinématique inverse (v, ω) → vitesses de roues, saturées comme les moteurs TT."""
        left = (self.v_cmd - self.w_cmd * L_CMD / 2) / self.radius
        right = (self.v_cmd + self.w_cmd * L_CMD / 2) / self.radius
        left = max(-WHEEL_MAX, min(WHEEL_MAX, left))
        right = max(-WHEEL_MAX, min(WHEEL_MAX, right))
        for a in self.act_left:
            self.d.ctrl[a] = left
        for a in self.act_right:
            self.d.ctrl[a] = right

    def pose(self) -> tuple[float, float, float]:
        """Pose vraie (x, y, θ), même formule que DiffDriveSim::truePose()."""
        x, y, _, qw, qx, qy, qz = self.d.qpos[:7]
        theta = math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
        return float(x), float(y), theta

    def speed(self) -> float:
        """Vitesse vraie du châssis le long de son cap (m/s)."""
        _, _, theta = self.pose()
        vx, vy = self.d.qvel[0], self.d.qvel[1]
        return float(vx * math.cos(theta) + vy * math.sin(theta))

    def hud(self) -> tuple[str, str]:
        x, y, theta = self.pose()
        # Texte en ASCII seulement : la police du viewer MuJoCo n'a ni accents, ni ω, ni °.
        labels = "\n".join(["commande v", "commande w", "vitesse vraie", "x", "y", "cap",
                            "temps", "", "fleches", "Suppr", "Backspace", "Espace"])
        values = "\n".join([f"{self.v_cmd:+.2f} m/s", f"{self.w_cmd:+.2f} rad/s",
                            f"{self.speed():+.2f} m/s", f"{x:+.2f} m", f"{y:+.2f} m",
                            f"{math.degrees(theta):+.0f} deg", f"{self.d.time:.1f} s", "",
                            "piloter", "arret", "depart", "pause"])
        return labels, values


def run_viewer(model: mujoco.MjModel, data: mujoco.MjData, duration: float | None) -> None:
    import mujoco.viewer

    robot = Robot(model, data)
    max_steps_per_frame = round(0.1 / model.opt.timestep)  # au plus 0,1 s simulée par image
    with mujoco.viewer.launch_passive(model, data, key_callback=robot.on_key) as viewer:
        with viewer.lock():
            # Caméra qui suit le robot, vue de derrière et de haut.
            viewer.cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING
            viewer.cam.trackbodyid = robot.chassis
            viewer.cam.distance, viewer.cam.elevation, viewer.cam.azimuth = 1.1, -25.0, 180.0
        frames = 0
        wall_start = time.perf_counter()
        # Temps réel : data.time doit suivre l'horloge murale ; wall_ref = instant où t sim = 0.
        wall_ref = wall_start
        while viewer.is_running():
            now = time.perf_counter()
            with viewer.lock():
                if robot.reset_requested:
                    mujoco.mj_resetData(model, data)
                    mujoco.mj_forward(model, data)
                    robot.reset_requested = False
                    wall_ref = now
                if robot.paused:
                    wall_ref = now - data.time  # le temps simulé ne rattrape pas la pause
                else:
                    robot.apply()
                    steps = 0
                    while data.time < now - wall_ref and steps < max_steps_per_frame:
                        mujoco.mj_step(model, data)
                        steps += 1
                    if steps == max_steps_per_frame:
                        wall_ref = now - data.time  # PC trop lent : on ralentit sans saccade
                labels, values = robot.hud()
            # set_texts et sync prennent eux-mêmes le verrou du viewer : les appeler à
            # l'intérieur de « with viewer.lock() » bloque la fenêtre (« ne répond pas »).
            viewer.set_texts((mujoco.mjtFontScale.mjFONTSCALE_150,
                              mujoco.mjtGridPos.mjGRID_TOPLEFT, labels, values))
            viewer.sync()
            frames += 1
            if duration is not None and time.perf_counter() - wall_start > duration:
                print(f"OK : {frames} images en {duration:.0f} s, t simulé = {data.time:.1f} s")
                break
            time.sleep(0.005)  # laisse respirer la fenêtre (≈ 100 images/s au plus)


def run_check(model: mujoco.MjModel, data: mujoco.MjData) -> None:
    """Rejoue une suite d'appuis de touches sans fenêtre et vérifie le comportement."""
    robot = Robot(model, data)

    def simulate(seconds: float) -> None:
        for _ in range(round(seconds / model.opt.timestep)):
            robot.apply()
            mujoco.mj_step(model, data)

    for _ in range(6):
        robot.on_key(KEY_UP)  # 0,30 m/s
    simulate(3.0)
    x, y, theta = robot.pose()
    assert abs(robot.speed() - 0.30) < 0.02, f"vitesse {robot.speed():.3f} au lieu de 0,30"
    assert x > 0.7 and abs(y) < 0.01 and abs(theta) < 0.01, f"ligne droite ratée : {x, y, theta}"

    for _ in range(5):
        robot.on_key(KEY_LEFT)  # ω = 1,0 rad/s
    simulate(1.5)  # mise en régime du virage
    theta_a = robot.pose()[2]
    simulate(0.2)
    w_real = math.remainder(robot.pose()[2] - theta_a, 2 * math.pi) / 0.2
    assert 0.8 < w_real < 1.2, f"virage à gauche : ω réel {w_real:.2f} rad/s pour 1,00 demandé"

    robot.on_key(KEY_DELETE)
    simulate(1.0)
    assert abs(robot.speed()) < 0.01, f"pas arrêté : {robot.speed():.3f} m/s"
    print(f"OK : ligne droite x = {x:.2f} m à 0,30 m/s ; virage ω réel = {w_real:.2f} rad/s "
          f"pour 1,00 demandé ; arrêt.")


def main() -> None:
    sys.stdout.reconfigure(encoding="utf-8")  # accents lisibles dans la console Windows
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--check", action="store_true", help="vérifie sans ouvrir de fenêtre")
    parser.add_argument("--duration", type=float, default=None,
                        help="ferme le viewer après N secondes (test de non-blocage)")
    args = parser.parse_args()
    model = mujoco.MjModel.from_xml_path(str(MODEL))
    data = mujoco.MjData(model)
    if args.check:
        run_check(model, data)
    else:
        run_viewer(model, data, args.duration)


if __name__ == "__main__":
    main()
