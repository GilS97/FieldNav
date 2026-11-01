# ADR-0004 — Gel de FieldTelemetry, sensor_utils recréé dans FieldNav

## Statut

Accepté — 31 octobre 2026 (gel décidé le 20 octobre 2026)

## Contexte

FieldTelemetry est un dépôt séparé qui sert de fil rouge à un cursus C++ en 9 étapes
(`ROADMAP.md`). Il en est à l'étape 1 : `reading.hpp`, `stats.hpp` et `report.hpp` sont presque
vides, et le dépôt GitHub ne compte qu'un commit. Le finir prendrait plusieurs mois.

Le cursus présente `sensor_utils` comme le jalon de l'étape 5 : une bibliothèque header-only en
C++23 (concepts, `std::span`, ranges, `std::expected`). Le plan prévoyait de l'importer dans
FieldNav. Le 31 octobre, son code est introuvable : il n'est ni dans le dépôt local de
FieldTelemetry, ni ailleurs sur le disque, ni dans WSL, ni sur GitHub.

D'ici le 23 novembre, la feature cible de FieldNav n'a besoin que de deux statistiques : la
RMS de l'erreur latérale (métrique de succès < 15 cm) et son maximum (légendes des figures).

## Décision

- Geler FieldTelemetry : plus de développement avant juin 2027, dépôt tagué `freeze-2026-10` et
  archivé sur GitHub, avec un README qui renvoie vers FieldNav.
- Recréer `sensor_utils` dans `libs/sensor_utils`, comme module de FieldNav et non comme
  sous-projet : il n'y a rien à importer.
- Limiter le module à `rms()` et `maxAbs()`. Une fonction n'y entre qu'avec un test et un usage
  dans la feature cible.
- Compiler le module en C++20 (cible INTERFACE, `cxx_std_20`), comme ADR-0001 l'autorise pour un
  module ; le reste de FieldNav reste en C++17. Le test ctest `sensor_utils.no_cxx23` empêche le
  retour de constructions postérieures à C++20.

## Conséquences

- ✅ Module minimal, couvert par 9 tests et compilé par la CI de FieldNav : une seule CI à
  maintenir.
- ✅ FieldNav ne dépend d'aucun dépôt externe.
- ⚠️ Les étapes 3 à 5 du cursus (concepts, ranges, `std::expected`) ne sont plus pratiquées.
  Mitigation : reprise de FieldTelemetry en juin 2027 ; d'ici là, ces outils n'entrent dans
  FieldNav que si la feature en a besoin.
- ⚠️ La tâche du 9 novembre prévoyait d'écrire le `TelemetryWriter` « dans sensor_utils (ou à
  côté) ». Mitigation : il ira dans `src/telemetry/`, comme le prévoit l'architecture visée, pour
  garder `sensor_utils` limité aux statistiques.
