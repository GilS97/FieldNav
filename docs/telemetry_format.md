# Format de télémétrie FieldNav

Format unique pour la simulation (`fieldnav_run`) et, en janvier 2027, pour le robot réel
(ESP8266 → MQTT). Il ne contient **aucun champ propre à la simulation** : sur le robot réel, les
colonnes de vérité terrain (`*_true`, `lat_err`) sont simplement vides.

Écrit par `fieldnav::telemetry::TelemetryWriter` (`src/telemetry/`), lu par `scripts/plot_run.py`.

## Repère et unités

- Origine : départ du rang. `x` vers l'avant le long du rang, `y` vers la gauche.
- Distances en mètres, angles en radians dans [−π, π], temps en secondes depuis le départ.
- Fréquence : **10 Hz** (une ligne toutes les 0,1 s), comme le flux MQTT prévu sur le robot réel.

## Colonnes

| Colonne     | Unité | Contenu                                                                      |
|-------------|-------|------------------------------------------------------------------------------|
| `t_s`       | s     | temps depuis le départ                                                       |
| `x_true`    | m     | position vraie (vérité terrain de la simulation ; vide sur le robot réel)    |
| `y_true`    | m     | idem                                                                         |
| `th_true`   | rad   | cap vrai                                                                     |
| `x_est`     | m     | position estimée utilisée par le contrôleur (vérité, odométrie ou fusion)    |
| `y_est`     | m     | idem                                                                         |
| `th_est`    | rad   | cap estimé                                                                   |
| `x_gps`     | m     | dernière mesure GPS reçue depuis la ligne précédente ; vide s'il n'y en a pas |
| `y_gps`     | m     | idem                                                                         |
| `gps_valid` | 0 / 1 | 1 si une mesure GPS est arrivée depuis la ligne précédente                   |
| `v_cmd`     | m/s   | vitesse commandée par le pure pursuit                                        |
| `w_cmd`     | rad/s | vitesse de rotation commandée (positive = vers la gauche)                    |
| `lat_err`   | m     | erreur latérale VRAIE par rapport à la ligne du rang (positive = à gauche)   |

Précision : 3 décimales pour `t_s`, 4 pour les positions, les vitesses et `lat_err`, 5 pour les
angles. Mêmes paramètres et même seed = fichier identique octet pour octet (fins de ligne LF).

## CSV (`.csv`)

Une ligne d'en-tête, puis une ligne par échantillon. Une valeur absente est un champ vide.

```csv
t_s,x_true,y_true,th_true,x_est,y_est,th_est,x_gps,y_gps,gps_valid,v_cmd,w_cmd,lat_err
0.100,0.0090,0.0000,0.00000,0.0091,0.0000,0.00002,,,0,0.3000,0.0000,0.0000
0.200,0.0370,0.0000,0.00000,0.0372,-0.0001,-0.00011,1.0315,-0.4021,1,0.3000,0.0007,0.0000
```

## JSON Lines (`.jsonl`)

Un objet JSON par ligne, mêmes clés que le CSV. Une valeur absente vaut `null`. C'est le format
des messages MQTT du robot réel.

```json
{"t_s":0.100,"x_true":0.0090,"y_true":0.0000,"th_true":0.00000,"x_est":0.0091,"y_est":0.0000,"th_est":0.00002,"x_gps":null,"y_gps":null,"gps_valid":0,"v_cmd":0.3000,"w_cmd":0.0000,"lat_err":0.0000}
```

Le format est choisi d'après l'extension du fichier (`--out run.csv` ou `--out run.jsonl`).
