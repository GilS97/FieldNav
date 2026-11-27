"""Rejoue un run FieldNav dans MuJoCo pour en faire une image ou une animation (README, vidéo).

Usage (depuis la racine du dépôt) :
    .venv/Scripts/python scripts/render_run.py RUN.csv --still 30 --out docs/img/robot_rang.png
    .venv/Scripts/python scripts/render_run.py RUN.csv --gif docs/img/fieldnav.gif

Décisions non évidentes :
1. On REJOUE la trajectoire vraie enregistrée (x_true, y_true, th_true) au lieu de relancer la
   simulation : l'image montre exactement le run dont parlent les chiffres et les figures, et le
   rendu ne dépend ni du C++ ni de la seed.
2. Les roues sont animées à partir de la distance parcourue (angle = s / r) : la télémétrie ne
   contient pas la vitesse des roues, et c'est le seul mouvement visible qu'il faut restituer.
3. Le GIF est accéléré (×7 par défaut : 170 s de run → environ 24 s) et réduit à 10 images/s et
   400 px de large, 64 couleurs, pour rester sous 5 Mo (exigence du README, CLAUDE.md).
"""

import argparse
import math
import pathlib
import sys

import mujoco
import numpy as np
import pandas as pd
from PIL import Image

ROOT = pathlib.Path(__file__).resolve().parents[1]
WHEEL_RADIUS = 0.0325


def load_scene(scene: str) -> tuple[mujoco.MjModel, mujoco.MjData]:
    model = mujoco.MjModel.from_xml_path(str(ROOT / "scenes" / f"{scene}.xml"))
    return model, mujoco.MjData(model)


def set_pose(model, data, x: float, y: float, th: float, s: float) -> None:
    """Place le châssis (freejoint) et fait tourner les roues d'après la distance parcourue."""
    data.qpos[0:3] = [x, y, WHEEL_RADIUS]
    data.qpos[3:7] = [math.cos(th / 2), 0.0, 0.0, math.sin(th / 2)]
    for name in ("j_wheel_fl", "j_wheel_rl", "j_wheel_fr", "j_wheel_rr"):
        data.qpos[model.joint(name).qposadr[0]] = s / WHEEL_RADIUS
    mujoco.mj_forward(model, data)


def make_camera(model, distance: float, elevation: float, azimuth: float) -> mujoco.MjvCamera:
    cam = mujoco.MjvCamera()
    cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING
    cam.trackbodyid = model.body("chassis").id
    cam.distance, cam.elevation, cam.azimuth = distance, elevation, azimuth
    return cam


def frames_at(df: pd.DataFrame, times: np.ndarray) -> list[tuple[float, float, float, float]]:
    """(x, y, θ, s) interpolés aux instants demandés ; s = distance parcourue cumulée."""
    seg = np.hypot(np.diff(df["x_true"]), np.diff(df["y_true"]))
    s = np.concatenate([[0.0], np.cumsum(seg)])
    th = np.unwrap(df["th_true"].to_numpy())
    t = df["t_s"].to_numpy()
    return [(float(np.interp(ti, t, df["x_true"])), float(np.interp(ti, t, df["y_true"])),
             float(np.interp(ti, t, th)), float(np.interp(ti, t, s))) for ti in times]


def main() -> int:
    sys.stdout.reconfigure(encoding="utf-8")
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("csv", type=pathlib.Path, help="télémétrie CSV d'un run")
    p.add_argument("--scene", default="row_straight", help="scène de scenes/ (sans .xml)")
    p.add_argument("--still", type=float, help="instant (s) de l'image fixe")
    p.add_argument("--gif", type=pathlib.Path, help="fichier GIF à produire")
    p.add_argument("--out", type=pathlib.Path, help="fichier PNG de l'image fixe")
    p.add_argument("--speed", type=float, default=7.0, help="accélération du GIF (défaut ×7)")
    p.add_argument("--fps", type=int, default=10)
    p.add_argument("--width", type=int, default=400)
    args = p.parse_args()

    df = pd.read_csv(args.csv)
    model, data = load_scene(args.scene)

    if args.still is not None:
        w, h = 1280, 720
        renderer = mujoco.Renderer(model, height=h, width=w)
        (x, y, th, s), = frames_at(df, np.array([args.still]))
        set_pose(model, data, x, y, th, s)
        renderer.update_scene(data, camera=make_camera(model, 2.2, -22, 155))
        out = args.out or args.csv.with_suffix(".png")
        out.parent.mkdir(parents=True, exist_ok=True)
        Image.fromarray(renderer.render()).save(out)
        print(f"écrit : {out}")

    if args.gif:
        w = args.width
        h = int(w * 9 / 16) // 2 * 2
        renderer = mujoco.Renderer(model, height=h, width=w)
        duration = float(df["t_s"].iloc[-1])
        times = np.arange(0.0, duration, args.speed / args.fps)
        cam = make_camera(model, 2.4, -24, 160)
        images = []
        for x, y, th, s in frames_at(df, times):
            set_pose(model, data, x, y, th, s)
            renderer.update_scene(data, camera=cam)
            images.append(Image.fromarray(renderer.render()).convert(
                "P", palette=Image.Palette.ADAPTIVE, colors=64))
        args.gif.parent.mkdir(parents=True, exist_ok=True)
        images[0].save(args.gif, save_all=True, append_images=images[1:],
                       duration=round(1000 / args.fps), loop=0, optimize=True)
        size = args.gif.stat().st_size / 1e6
        print(f"écrit : {args.gif} ({len(images)} images, {len(images) / args.fps:.0f} s, "
              f"{size:.1f} Mo)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
