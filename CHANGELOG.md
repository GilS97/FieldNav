# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

## [0.1.0] - 2026-12-01

Premier livrable : le robot suit un rang de culture en boucle fermée à partir d'une pose estimée
(ADR 0005).

### Ajouté

- Simulation MuJoCo 3.16 du châssis LAFVIN 4WD (skid-steer), téléchargée par CMake ; scène de rang
  droit générée par `scripts/gen_row_scene.py` et sa trajectoire de référence.
- Odométrie différentielle (Euler), cinématique roues ↔ (v, ω), bruit des roues ±2 % seedé.
- GPS simulé : bruit blanc, biais en marche aléatoire, pertes, trou GNSS.
- Fusion complémentaire odométrie + GPS (α = 0,05, x et y seulement).
- `Path` (erreur latérale signée, point visé) et pure pursuit.
- `fieldnav_run` : mission complète, codes retour 0 / 1 / 2 ; `scripts/run_matrix.sh`.
- Télémétrie CSV et JSON Lines (`docs/telemetry_format.md`) ; figures `scripts/plot_run.py` ;
  rendu `scripts/render_run.py`.
- `libs/sensor_utils` recréé en C++20 (RMS, max) ; ADR 0004 et 0005.
- CI : 4 builds, sanitizers, format, matrice réduite de missions.

### Résultat

- Seed 42 : erreur latérale RMS 14,4 cm en fusion, 34,8 cm puis sortie de rang en odométrie seule.
- 11 seeds : médiane 18,1 cm, objectif de 15 cm non atteint en médiane (ADR 0005, « Résultat
  mesuré ») ; aucune sortie de rang.

[Unreleased]: https://github.com/GilS97/FieldNav/compare/v0.1.0...HEAD
[0.1.0]: https://github.com/GilS97/FieldNav/releases/tag/v0.1.0
