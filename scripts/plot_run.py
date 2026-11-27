"""Figures d'un run FieldNav (télémétrie de docs/telemetry_format.md), au style du mémoire.

Usage (depuis la racine du dépôt) :
    .venv/Scripts/python scripts/plot_run.py results/2026-11-27/row_straight_fusion_seed42.csv
    .venv/Scripts/python scripts/plot_run.py --compare results/2026-11-27/row_straight_*_seed42.csv

Sorties (dans <dossier du CSV>/fig/, ou --out DOSSIER), chacune en PNG 200 dpi ET en PDF :
    <run>_dessus     vue de dessus : trajectoire vraie, estimée, mesures GPS, rangées de plants
    <run>_lateral    erreur latérale vraie en fonction du temps
    compare_lateral  (--compare) erreurs latérales de plusieurs runs superposées

Style commun (CLAUDE.md) : 16 × 9 cm, police 9 pt, RMS et max dans la légende, trou GNSS grisé.

Décisions non évidentes :
1. Couleurs FIXES par source de pose (vérité = bleu, odométrie = orange, fusion = aqua : les trois
   premiers emplacements, dans l'ordre, de la palette de référence lisible par les daltoniens),
   et un style de trait par source (plein, tirets, tirets-points) : une même source garde la même
   couleur dans toutes les figures du mémoire, et les courbes restent distinctes imprimées en
   noir et blanc.
2. Le trou GNSS est DÉTECTÉ dans la télémétrie (aucune mesure GPS pendant plus de 3 s), pas lu
   dans les options du run : la même règle marchera sur les données du robot réel en janvier.
   Les pertes ordinaires (20 %) ne durent que quelques dixièmes de seconde.
3. Les RMS sont recalculés depuis le CSV (10 Hz) : ils peuvent différer au millimètre près de
   ceux qu'affiche fieldnav_run (calculés à 500 Hz). La figure dit ce que montre le fichier.
"""

import argparse
import math
import pathlib
import re
import sys

import matplotlib

matplotlib.use("Agg")  # pas de fenêtre : on écrit des fichiers
import matplotlib.pyplot as plt  # noqa: E402
import pandas as pd  # noqa: E402

CM = 1 / 2.54
FIGSIZE = (16 * CM, 9 * CM)
INK = "#2b2b2b"       # texte et trajectoire vraie dans la vue de dessus
MUTED = "#8a8a85"     # axes, mesures GPS, ligne médiane
GAP_FILL = "#d9d9d4"  # trou GNSS
PLANT = "#9cc79a"     # rangées de plants
SOURCES = {           # couleur et style de trait par source de pose (fixes)
    "truth": ("#2a78d6", "-", "vérité"),
    "odom": ("#eb6834", "--", "odométrie seule"),
    "fusion": ("#1baf7a", "-.", "fusion odométrie + GPS"),
}
DEFAULT_SOURCE = ("#2a78d6", "-", None)
ROW_HALF_SPACING = 0.75  # m, demi-écart entre rangées (scripts/gen_row_scene.py)
OUTAGE_MIN_S = 3.0

plt.rcParams.update({
    "font.size": 9, "axes.titlesize": 9, "axes.labelsize": 9, "legend.fontsize": 8,
    "xtick.labelsize": 8, "ytick.labelsize": 8, "axes.edgecolor": MUTED,
    "axes.labelcolor": INK, "xtick.color": INK, "ytick.color": INK, "axes.grid": True,
    "grid.color": "#e6e6e2", "grid.linewidth": 0.6, "axes.spines.top": False,
    "axes.spines.right": False, "legend.frameon": False, "lines.linewidth": 1.4,
    "savefig.dpi": 200, "pdf.fonttype": 42,
})


def source_of(path: pathlib.Path) -> str:
    """« row_straight_fusion_seed42.csv » → « fusion » (sinon le nom du fichier)."""
    m = re.search(r"_(truth|odom|fusion)(?:_|$)", path.stem)
    return m.group(1) if m else path.stem


def style_of(path: pathlib.Path) -> tuple[str, str, str]:
    color, ls, label = SOURCES.get(source_of(path), DEFAULT_SOURCE)
    return color, ls, label or path.stem


def load(path: pathlib.Path) -> pd.DataFrame:
    df = pd.read_csv(path)
    missing = {"t_s", "x_true", "y_true", "x_est", "y_est", "gps_valid", "lat_err"} - set(df.columns)
    if missing:
        raise SystemExit(f"{path} : colonnes manquantes {sorted(missing)} (voir docs/telemetry_format.md)")
    return df


def outages(df: pd.DataFrame) -> list[tuple[float, float]]:
    """Intervalles sans aucune mesure GPS pendant plus de OUTAGE_MIN_S secondes."""
    t_fix = df.loc[df["gps_valid"] == 1, "t_s"].tolist()
    bounds = [0.0] + t_fix + [float(df["t_s"].iloc[-1])]
    return [(a, b) for a, b in zip(bounds, bounds[1:]) if b - a > OUTAGE_MIN_S]


def rms_max(err: pd.Series) -> tuple[float, float]:
    e = err.dropna()
    return math.sqrt((e**2).mean()), e.abs().max()


def shade_outages(ax, windows) -> None:
    for i, (a, b) in enumerate(windows):
        ax.axvspan(a, b, color=GAP_FILL, alpha=0.6, lw=0, zorder=0,
                   label=f"trou GNSS ({b - a:.0f} s)" if i == 0 else None)


def save(fig, out_dir: pathlib.Path, name: str) -> None:
    out_dir.mkdir(parents=True, exist_ok=True)
    fig.tight_layout()
    for ext in ("png", "pdf"):
        fig.savefig(out_dir / f"{name}.{ext}")
    plt.close(fig)
    print(f"écrit : {out_dir / name}.png + .pdf")


def plot_top_view(path: pathlib.Path, df: pd.DataFrame, out_dir: pathlib.Path) -> None:
    color, ls, label = style_of(path)
    fig, ax = plt.subplots(figsize=FIGSIZE)
    x_end = max(df["x_true"].max(), 1.0)
    for y in (ROW_HALF_SPACING, -ROW_HALF_SPACING):
        ax.plot([0, x_end], [y, y], color=PLANT, lw=3, solid_capstyle="butt", zorder=1,
                label="rangées de plants" if y > 0 else None)
    ax.plot([0, x_end], [0, 0], color=MUTED, lw=0.8, ls=":", zorder=1, label="ligne du rang")
    gps = df[df["gps_valid"] == 1]
    ax.scatter(gps["x_gps"], gps["y_gps"], s=4, color=MUTED, alpha=0.35, lw=0, zorder=2,
               label="mesures GPS")
    ax.plot(df["x_true"], df["y_true"], color=INK, lw=1.6, zorder=3, label="trajectoire vraie")
    if source_of(path) != "truth":
        ax.plot(df["x_est"], df["y_est"], color=color, ls=ls, zorder=4, label=f"estimée ({label})")
    ax.set_xlabel("x le long du rang (m)")
    ax.set_ylabel("y (m)")
    ax.set_ylim(-1.6, 1.6)
    ax.set_title(f"Vue de dessus — {label} (échelle latérale dilatée)", loc="left")
    ax.legend(loc="upper center", ncol=3, bbox_to_anchor=(0.5, -0.2))
    save(fig, out_dir, f"{path.stem}_dessus")


def plot_lateral(path: pathlib.Path, df: pd.DataFrame, out_dir: pathlib.Path) -> None:
    color, ls, label = style_of(path)
    rms, mx = rms_max(df["lat_err"])
    fig, ax = plt.subplots(figsize=FIGSIZE)
    shade_outages(ax, outages(df))
    ax.axhline(0, color=MUTED, lw=0.8)
    ax.plot(df["t_s"], df["lat_err"] * 100, color=color, ls=ls,
            label=f"{label} — RMS {rms * 100:.1f} cm, max {mx * 100:.1f} cm")
    ax.set_xlabel("temps (s)")
    ax.set_ylabel("erreur latérale (cm)")
    ax.set_title("Erreur latérale vraie (positive = à gauche de la ligne)", loc="left")
    ax.legend(loc="best")
    save(fig, out_dir, f"{path.stem}_lateral")


def plot_compare(paths: list[pathlib.Path], out_dir: pathlib.Path) -> None:
    order = {"truth": 0, "odom": 1, "fusion": 2}
    paths = sorted(paths, key=lambda p: order.get(source_of(p), 9))
    fig, ax = plt.subplots(figsize=FIGSIZE)
    frames = [(p, load(p)) for p in paths]
    # Trou GNSS : celui du run de fusion s'il y en a un (même GPS pour tous les runs d'une seed).
    ref = next((df for p, df in frames if source_of(p) == "fusion"), frames[0][1])
    shade_outages(ax, outages(ref))
    ax.axhline(0, color=MUTED, lw=0.8)
    for p, df in frames:
        color, ls, label = style_of(p)
        rms, mx = rms_max(df["lat_err"])
        ax.plot(df["t_s"], df["lat_err"] * 100, color=color, ls=ls,
                label=f"{label} — RMS {rms * 100:.1f} cm, max {mx * 100:.1f} cm")
        print(f"{p.name} : RMS {rms * 100:.1f} cm, max {mx * 100:.1f} cm, {df['t_s'].iloc[-1]:.0f} s")
    ax.set_xlabel("temps (s)")
    ax.set_ylabel("erreur latérale (cm)")
    ax.set_title("Erreur latérale vraie selon la pose donnée au pilote", loc="left")
    ax.legend(loc="upper left")
    save(fig, out_dir, "compare_lateral")


def main() -> int:
    sys.stdout.reconfigure(encoding="utf-8")
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("csv", nargs="+", type=pathlib.Path, help="télémétrie CSV (un ou plusieurs)")
    parser.add_argument("--compare", action="store_true", help="superposer les erreurs latérales")
    parser.add_argument("--out", type=pathlib.Path, help="dossier des figures (défaut : <csv>/fig)")
    args = parser.parse_args()

    # PowerShell ne développe pas « row_straight_*.csv » (bash le fait) : on développe ici les
    # motifs restés tels quels, pour que la même commande marche dans les deux shells.
    csvs: list[pathlib.Path] = []
    for p in args.csv:
        if any(c in str(p) for c in "*?["):
            matches = sorted(p.parent.glob(p.name))
            if not matches:
                print(f"aucun fichier ne correspond à : {p}", file=sys.stderr)
                return 2
            csvs += matches
        elif not p.exists():
            print(f"introuvable : {p}", file=sys.stderr)
            return 2
        else:
            csvs.append(p)
    out_dir = args.out or csvs[0].parent / "fig"
    if args.compare:
        plot_compare(csvs, out_dir)
    else:
        for p in csvs:
            df = load(p)
            plot_top_view(p, df, out_dir)
            plot_lateral(p, df, out_dir)
    return 0


if __name__ == "__main__":
    sys.exit(main())
