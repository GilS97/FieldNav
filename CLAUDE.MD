# CLAUDE.md — FieldNav

À déposer à la racine du dépôt FieldNav. Lis ce fichier en entier, puis `docs/adr/` et le `README.md`, avant
toute modification. Ce fichier dit **ce qu'on construit d'ici le 23 novembre 2026 et ce qu'on ne construit pas** ;
le code existant dit comment le dépôt est organisé — adapte-toi à lui, ne le réorganise pas.

## Ce que c'est

Stack open-source de localisation/navigation pour robots agricoles d'extérieur : C++17/20, ROS 2, simulation
MuJoCo (Isaac Lab envisagé, **hors périmètre jusqu'en 2027**). Projet capstone de Gil (M2 Réseaux et Objets
Connectés, Perpignan), qui devient le support de son mémoire de M2 et de son dossier de thèse (octobre 2027).
État au 20 octobre 2026 : squelette de 57 fichiers, 14/14 gtests, CI GitHub Actions, 3 ADR ; **aucune feature de
bout en bout**.

FieldTelemetry (dépôt séparé) est gelé ; sa seule brique réutilisable, `sensor_utils` (header-only, C++23),
devient un module de FieldNav abaissé en C++20.

## La feature cible — la seule définition de « fini » jusqu'au 23 novembre

Un robot différentiel simulé suit une ligne de culture en boucle fermée **à partir d'une pose estimée** (odométrie
bruitée + GPS bruité fusionnés), exporte sa télémétrie dans un format documenté, et il existe une vidéo de 30 s et
un README avec une figure d'erreur.

Métrique de succès : erreur latérale RMS < 15 cm sur 50 m, avec GPS à σ = 1,0 m, 20 % de pertes, et un trou GNSS
de 10 s au milieu du rang.

**Hors périmètre, explicitement** (refuser poliment et renvoyer à ce paragraphe si on te le demande) : lidar,
vision, SLAM, détection réelle du rang, demi-tour en bout de rang, multi-robots, Isaac Lab, Kalman/UKF/graphes de
facteurs avant janvier, refonte de l'architecture, nouveau framework de build, nouvelles dépendances lourdes.

## Les quatre jalons

| Date | Jalon | On sait que c'est fait quand… |
|---|---|---|
| 26 oct | Fondations | FieldTelemetry archivé ; `libs/sensor_utils` compile en C++20 dans FieldNav, ses tests tournent dans ctest ; ADR 0004 (fusion) et ADR 0005 (feature cible) commités ; module `odometry` avec API + tests triviaux ; CI verte |
| 2 nov | Capteurs | Odométrie alimentée par la sim (bruit ±2 % sur les roues) ; `GpsSim` (biais + marche aléatoire + pertes, seed fixée, tests statistiques) ; fusion complémentaire ; scène de rang (droite + courbe) + CSV de référence ; RMS odo / gps / fusion imprimés en fin de run |
| 9 nov | Contrôle | `Path` (erreur latérale signée, lookahead) ; `PurePursuit` ; suivi sur vérité terrain puis sur estimée ; exécutable `fieldnav_run` avec options ; `scripts/run_matrix.sh` ; matrice réduite dans la CI |
| 16 nov | Livrable | `docs/telemetry_format.md` + `TelemetryWriter` CSV/JSONL ; `scripts/plot_run.py` (trajectoires, erreur latérale, mode `--compare`) ; README avec GIF + tableau RMS + quickstart vérifié sur clone frais ; tag `v0.1.0` |
| 23 nov | Vidéo | MP4 30 s en release GitHub, GIF < 5 Mo dans le README |

Un jalon = une issue GitHub « Jalon <date> ». Fermer l'issue = jalon atteint. Les détails étape par étape de
chaque session se trouvent dans `cap2027-details-taches.json` (projet « fieldnav », fichier fourni par Gil) :
quand Gil nomme une tâche par son titre, va chercher ses `etapes` et son `critere de fini` là-dedans.

## Architecture visée (à placer là où le dépôt range déjà ce genre de choses)

```
libs/sensor_utils/          importé de FieldTelemetry, INTERFACE library, cxx_std_20
src/odometry/               Pose2D {x,y,theta} ; DiffDriveOdometry::update(v, w, dt) — Euler, θ normalisé [-π, π]
src/sensors/gps_sim.hpp     GpsSim(seed, sigma_noise=1.0, sigma_rw=0.02, p_loss=0.2, rate_hz=5) → optional<GpsFix>
                            + mode « trou GNSS » [t1, t2]
src/estimation/             ComplementaryFuser : predict(v,w,dt) / correct(GpsFix) ; corrige x,y seulement, α ≈ 0.1–0.3
src/path/path.hpp           Path : closestIndex (recherche locale), lateralError (signée), lookahead(d)
src/control/pure_pursuit.hpp  Cmd compute(pose, path, v_des, lookahead) ; κ = 2 sin α / L_d ; saturations
src/telemetry/              TelemetryWriter (CSV + JSON Lines), colonnes fixées par docs/telemetry_format.md
apps/fieldnav_run.cpp       --scene --reference --pose-source {truth,odom,fusion} --seed --duration
                            --gps-sigma --gps-loss --gps-outage t1,t2 --out ; codes retour 0 ok / 1 sortie de rang / 2 config
scenes/row_straight.xml, row_curved.xml   générées par scripts/gen_row_scene.py (longueur, écartement 1,5 m, pas 0,3 m, courbure)
data/reference_*.csv        trajectoire médiane, un point tous les 10 cm
scripts/run_matrix.sh, scripts/plot_run.py, results/<date>/
docs/adr/0004-fusion-fieldtelemetry.md, 0005-feature-cible.md, docs/telemetry_format.md
```

Conversion roues ↔ (v, ω) : v = r(ωd + ωg)/2 ; ω = r(ωd − ωg)/L. Cinématique inverse pour appliquer une
commande. Repère : départ du rang = origine, x vers l'avant, angles en radians, temps en secondes.

Format télémétrie (colonnes CSV, mêmes clés en JSONL) :
`t_s, x_true, y_true, th_true, x_est, y_est, th_est, x_gps, y_gps, gps_valid, v_cmd, w_cmd, lat_err`.
Ce format sera réutilisé tel quel par un robot réel en janvier (ESP8266 → MQTT) : ne pas y mettre de champ
spécifique à la sim.

## Mode de travail — important

- **Une session = une tâche = 50 minutes** (ou 2 × 80 min le samedi). Gil arrive avec un titre de tâche ; tu ne
  commences pas la suivante si la première est finie en avance : tu nettoies (tests, README, commit).
- **Gil préfère un échafaudage guidé avec des points de contrôle, pas des solutions complètes.** Par défaut :
  tu écris l'API (headers), les tests qui doivent passer et le câblage CMake/CI ; il écrit le cœur de la logique ;
  tu relis et proposes des corrections ciblées. Il peut te demander explicitement de tout écrire (« fais-le ») —
  alors fais-le, mais explique les 3 décisions non évidentes en commentaire.
- **Test d'abord, feature avant architecture.** Un module n'existe que s'il a un test. Pas de couche d'abstraction
  « pour plus tard ». Si une amélioration te démange (EKF, perception, refactor), écris-la en une ligne dans
  l'issue du jalon concerné, pas dans le code.
- **Reproductible** : tout aléa passe par un `std::mt19937` seedé injecté. Même seed = même CSV, octet pour octet.
- **Commit à chaque fin de session**, même sale, message en français au présent (« odometry: squelette + tests »).
  La CI doit rester verte sur `main` ; une branche par tâche si tu casses quelque chose.
- Langue : code et identifiants en anglais, commentaires, ADR, README et messages de commit en français.
- N'installe rien de nouveau sans le dire : MuJoCo, ROS 2, gtest, CMake, Python (pandas, matplotlib) suffisent.

## Pièges connus

- `sensor_utils` utilise des constructions C++23 (`std::expected`, `std::print`, deducing this…) : les remplacer
  par des équivalents C++20 plutôt que de monter tout FieldNav en C++23.
- L'odométrie par Euler dérive sur un cercle : le test « cercle » tolère 5 cm avec dt = 1e-3 ; ne pas « corriger »
  en passant à un intégrateur exact avant janvier.
- Le GPS ne donne pas le cap : la fusion ne corrige jamais θ.
- Le lookahead du pure pursuit est le seul réglage ; noter la valeur retenue dans le journal de Gil, pas en dur
  sans commentaire.
- Les figures servent au mémoire : même style partout (16 × 9 cm, 9 pt, PNG 200 dpi + PDF), RMS et max dans la
  légende, zones de trou GNSS grisées.

## Après le 23 novembre (pour information, ne pas commencer avant)

Décembre : expériences du mémoire (RMS vs σ_GPS, vs taux de pertes, vs durée du trou). Janvier : banc réel
(châssis 4 roues + ESP8266, même format de télémétrie, pure pursuit porté en C Arduino) et, seulement alors,
un EKF en option. L'article de M2 (soumissible le 31 mars 2027) s'appuie sur ces courbes.