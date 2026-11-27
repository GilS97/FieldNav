"""Génère une scène MuJoCo de rang de culture et sa trajectoire de référence.

Usage (depuis la racine du dépôt) :
    python scripts/gen_row_scene.py                       # rang droit de 50 m (par défaut)
    python scripts/gen_row_scene.py --radius 100 --name curved   # bonus : rang courbe, R = 100 m
    python scripts/gen_row_scene.py --check               # vérifie que les fichiers commités
                                                          # sont à jour (utilisé par ctest)

Sorties :
    scenes/row_<name>.xml        le robot (scenes/robot_diffdrive.xml, inclus) + deux rangées de plants
    data/reference_<name>.csv    la ligne médiane du rang, un point tous les 10 cm : s,x,y

Repère (CLAUDE.md) : départ du rang = origine, x vers l'avant, y vers la gauche, en mètres.
Le robot part de l'origine, cap 0, au milieu des deux rangées.

"""

import argparse
import math
import pathlib
import sys

ROOT = pathlib.Path(__file__).resolve().parents[1]


def centerline(length: float, step: float, radius: float) -> list[tuple[float, float, float]]:
    """Points (s, x, y) de la ligne médiane, de s = 0 à s = length inclus, tous les `step` m.

    radius = 0 : ligne droite ; radius > 0 : arc de cercle tournant à gauche.
    """
    points = []
    n = round(length / step)
    for i in range(n+1):
        s = i * step
        if radius ==0:
            x,y=s,0.0
        else:
            x = radius * math.sin(s/radius)
            y = radius * (1 - math.cos(s/radius))
        points.append((s,x,y))
    return points


def plants(length: float, spacing: float, pitch: float, radius: float) -> list[tuple[float, float]]:
    """Positions (x, y) des plants : deux rangées à ±spacing/2 de la ligne médiane, un plant tous
    les `pitch` m le long du rang, de s = 0 à s = length inclus."""

    positions = []
    n = round(length / pitch)
    for i in range(n+1):
        s = i * pitch
        if radius == 0:
            x,y = s,0.0
            nx,ny = 0.0,1.0
        else:
            x = radius * math.sin(s/radius)
            y = radius * (1 - math.cos(s/radius))
            nx,ny = -math.sin(s/radius), math.cos(s/radius)
        positions.append((x + spacing/2*nx, y + spacing/2*ny))
        positions.append((x - spacing/2*nx, y - spacing/2*ny))
    return positions



def plant_height(i: int) -> float:
    """Hauteur variée mais déterministe (pas d'aléa : même scène à chaque génération)."""
    return 0.12 + 0.06 * ((i * 37) % 11) / 10


def scene_xml(name: str, args: argparse.Namespace, points: list[tuple[float, float]]) -> str:
    lines = [
        "<!-- Généré par scripts/gen_row_scene.py : ne pas modifier à la main.",
        f"     Paramètres : length={args.length} m, spacing={args.spacing} m, pitch={args.pitch} m, "
        f"radius={args.radius} m. -->",
        f'<mujoco model="fieldnav_row_{name}">',
        '  <include file="robot_diffdrive.xml"/>',
        "  <asset>",
        '    <material name="plant" rgba="0.24 0.55 0.20 1" specular="0.1"/>',
        '    <material name="plant_top" rgba="0.36 0.68 0.26 1" specular="0.1"/>',
        "  </asset>",
        "  <worldbody>",
        "    <!-- Plants décoratifs (sans collision) : le robot roule entre les rangées. -->",
    ]
    for i, (x, y) in enumerate(points):
        h = plant_height(i)
        lines.append(
            f'    <geom name="plant_{i}" type="capsule" size="0.012 {h / 2:.3f}" '
            f'pos="{x:.4f} {y:.4f} {h / 2 + 0.025:.3f}" material="plant" '
            'contype="0" conaffinity="0" group="1" mass="0"/>'
        )
        # Feuillage : deux ellipsoïdes aplatis, légèrement décalés, pour un port de plant buissonnant.
        lines.append(
            f'    <geom type="ellipsoid" size="0.11 0.09 0.05" pos="{x:.4f} {y:.4f} {h + 0.02:.3f}" '
            'material="plant_top" contype="0" conaffinity="0" group="1" mass="0"/>'
        )
        lines.append(
            f'    <geom type="ellipsoid" size="0.07 0.08 0.06" pos="{x + 0.02:.4f} {y - 0.01:.4f} '
            f'{h + 0.08:.3f}" material="plant" contype="0" conaffinity="0" group="1" mass="0"/>'
        )
    lines += ["  </worldbody>", "</mujoco>", ""]
    return "\n".join(lines)


def reference_csv(rows: list[tuple[float, float, float]]) -> str:
    out = ["s,x,y"]
    out += [f"{s:.3f},{x:.4f},{y:.4f}" for s, x, y in rows]
    return "\n".join(out) + "\n"


def main() -> int:
    sys.stdout.reconfigure(encoding="utf-8")  # accents lisibles dans la console Windows
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--name", default="straight", help="suffixe des fichiers (straight, curved)")
    parser.add_argument("--length", type=float, default=50.0, help="longueur du rang (m)")
    parser.add_argument("--spacing", type=float, default=1.5, help="écart entre rangées (m)")
    parser.add_argument("--pitch", type=float, default=0.3, help="distance entre plants (m)")
    parser.add_argument("--radius", type=float, default=0.0, help="rayon de courbure (m), 0 = droit")
    parser.add_argument("--step", type=float, default=0.1, help="pas de la référence (m)")
    parser.add_argument("--check", action="store_true",
                        help="ne rien écrire, échouer si les fichiers commités diffèrent")
    args = parser.parse_args()

    rows = centerline(args.length, args.step, args.radius)
    pts = plants(args.length, args.spacing, args.pitch, args.radius)
    if not rows or not pts:
        print("centerline() ou plants() renvoie une liste vide : rien à écrire.")
        return 1

    outputs = {
        ROOT / "scenes" / f"row_{args.name}.xml": scene_xml(args.name, args, pts),
        ROOT / "data" / f"reference_{args.name}.csv": reference_csv(rows),
    }
    if args.check:
        stale = [p for p, text in outputs.items()
                 if not p.exists() or p.read_text(encoding="utf-8") != text]
        for p in stale:
            print(f"pas à jour : {p.relative_to(ROOT)} (relancer gen_row_scene.py)")
        return 1 if stale else 0

    for path, text in outputs.items():
        path.parent.mkdir(exist_ok=True)
        path.write_text(text, encoding="utf-8", newline="\n")
        print(f"écrit : {path.relative_to(ROOT)}")
    print(f"{len(rows)} points de référence, {len(pts)} plants.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
