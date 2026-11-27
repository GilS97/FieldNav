#!/usr/bin/env bash
# Lance la matrice de runs de FieldNav : 3 sources de pose (truth, odom, fusion) sur le rang
# droit, dans les conditions de la métrique (GPS σ = 1 m, 20 % de pertes, trou GNSS de 10 s au
# milieu du rang), et range la télémétrie dans results/<date>/.
#
# Usage : scripts/run_matrix.sh [DOSSIER_BUILD] [DUREE_MAX_S] [SEED]
#   scripts/run_matrix.sh build/debug          # matrice complète (rang entier, ≈ 170 s simulées)
#   scripts/run_matrix.sh build 10             # matrice réduite pour la CI (10 s simulées)
# Code retour : 2 si un run a une erreur de configuration ; 1 si la vérité ou la fusion sortent
# du rang. Une sortie de rang de l'odométrie seule est un RÉSULTAT attendu (sans GPS, elle dérive
# jusqu'aux plants), pas une erreur : elle est signalée mais ne fait pas échouer la matrice.
set -u

BUILD_DIR="${1:-build/debug}"
DURATION="${2:-300}"
SEED="${3:-42}"
ROOT="$(cd "$(dirname "$0")/.." && pwd)"

# fieldnav_run sous Linux (build/apps/fieldnav_run) ou Windows (…/fieldnav_run.exe).
RUN="$BUILD_DIR/apps/fieldnav_run"
[ -x "$RUN" ] || RUN="$RUN.exe"
if [ ! -x "$RUN" ]; then
    echo "fieldnav_run introuvable dans $BUILD_DIR/apps (compiler d'abord)" >&2
    exit 2
fi

OUT="$ROOT/results/$(date +%Y-%m-%d)"
mkdir -p "$OUT"

# Sous WSL, fieldnav_run.exe est un programme Windows : il ne comprend pas « /mnt/d/... ».
# On lui passe le chemin Windows (D:\...) ; sous Git Bash et Linux, rien à convertir.
native_path() {
    case "$RUN" in
        *.exe) if command -v wslpath >/dev/null 2>&1; then wslpath -w "$1"; return; fi ;;
    esac
    printf '%s' "$1"
}
status=0
for source in truth odom fusion; do
    file="$OUT/row_straight_${source}_seed${SEED}.csv"
    echo "== $source"
    "$RUN" --scene row_straight --pose-source "$source" --seed "$SEED" \
           --duration "$DURATION" --gps-outage 80,90 --out "$(native_path "$file")"
    code=$?
    if [ "$code" -eq 2 ]; then
        status=2
    elif [ "$code" -eq 1 ] && [ "$source" != "odom" ] && [ "$status" -eq 0 ]; then
        status=1
    elif [ "$code" -eq 1 ]; then
        echo "   (sortie de rang de l'odométrie seule : résultat attendu)"
    fi
done
echo "Télémétrie : $OUT"
exit "$status"
