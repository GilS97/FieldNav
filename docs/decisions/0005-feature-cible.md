# ADR-0005 — Feature cible : suivre un rang de culture à partir d'une pose estimée

## Statut

Accepté — 1er novembre 2026 (planning recalé d'une semaine le même jour)

## Contexte

Au 1er novembre 2026, FieldNav a un squelette de build, une CI verte et un module
`sensor_utils` (ADR-0004), mais aucune fonctionnalité de bout en bout. Le dépôt sert de support
au mémoire de M2 et au dossier de thèse : il faut une démonstration mesurable, reproductible et
montrable avant fin novembre, puis des expériences en décembre.

Le planning initial (jalons du 26 octobre au 23 novembre) a pris une semaine de retard : le
code de `sensor_utils` était perdu et la CI de `main` était rouge depuis mai.

## Décision

D'ici le 23 novembre 2026, FieldNav fait une seule chose :

- un robot différentiel simulé dans MuJoCo ;
- qui estime sa pose en fusionnant une odométrie bruitée (±2 % sur les roues) et un GPS bruité
  (filtre complémentaire, correction de x et y seulement) ;
- qui suit un rang de culture droit en boucle fermée (pure pursuit) à partir de cette estimée ;
- qui exporte sa télémétrie dans un format documenté (`docs/telemetry_format.md`), réutilisable
  tel quel par le robot réel en janvier ;
- qui est montré par une vidéo de 30 s et un README avec une figure d'erreur.

**Métrique de succès** : erreur latérale RMS < 15 cm sur 50 m, avec un GPS à σ = 1,0 m, 20 % de
pertes et un trou GNSS de 10 s au milieu du rang. Toute expérience est seedée : même seed, même
CSV.

**Jalons** (une issue GitHub « Jalon <date> » par jalon ; fermer l'issue = jalon atteint) :

| Date | Jalon | On sait que c'est fait quand… |
|---|---|---|
| 3 nov | Fondations | ADR 0004 et 0005 commités ; 4 issues ouvertes ; module `odometry` avec API et tests (ligne droite, quart de tour, cercle à 5 cm) ; CI verte |
| 9 nov | Capteurs | Odométrie alimentée par la sim ; `GpsSim` seedé avec tests statistiques ; fusion complémentaire ; scène de rang droit + CSV de référence ; RMS odo / gps / fusion imprimés en fin de run |
| 16 nov | Contrôle | `Path` et `PurePursuit` testés ; suivi sur vérité terrain puis sur fusion, rang droit ; `fieldnav_run` avec options ; `scripts/run_matrix.sh` ; matrice réduite dans la CI |
| 23 nov | Livrable | `TelemetryWriter` CSV ; `scripts/plot_run.py` ; README avec GIF, tableau RMS et quickstart vérifié sur clone frais ; tag `v0.1.0` ; MP4 de 30 s en release |

**Hors périmètre** jusqu'au 23 novembre : lidar, vision, SLAM, détection réelle du rang,
demi-tour en bout de rang, multi-robots, Isaac Lab, Kalman / UKF / graphes de facteurs (pas
avant janvier), refonte de l'architecture, nouveau framework de build, nouvelle dépendance
lourde.

## Conséquences

- ✅ Une définition de « fini » unique et chiffrée : chaque tâche se juge à son effet sur la
  métrique.
- ✅ Les courbes de décembre (RMS en fonction de σ GPS, du taux de pertes, de la durée du trou)
  réutilisent directement `fieldnav_run` et `run_matrix.sh`.
- ⚠️ Le rang courbe n'est plus garanti. Mitigation : il est ajouté au jalon Contrôle seulement si
  le suivi sur le rang droit est validé avant le 13 novembre ; sinon il passe en décembre.
- ⚠️ La sortie JSON Lines (pour MQTT) n'est plus dans un jalon. Mitigation : même clés que le CSV,
  ajoutée s'il reste du temps avant le 23 novembre, et au plus tard pour le banc réel de janvier.
- ⚠️ Le planning suppose environ une tâche de 50 minutes par jour jusqu'au 16 novembre.
  Mitigation : si un jalon glisse encore, la vidéo passe en décembre plutôt que de couper la
  feature.
