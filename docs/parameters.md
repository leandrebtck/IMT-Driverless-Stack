# Paramètres clés

## `yolo_lidar.py` — YOLO + LiDAR + Kalman

| Paramètre | Valeur | Description | Impact si modifié |
|---|---|---|---|
| `MIN_HITS` | 3 | Détections min avant publication d'un cône | ↑ = moins de faux positifs, ↓ = détection plus rapide |
| `MAX_MISSES` | 8 | Frames sans détection avant suppression du tracker | ↑ = trackers plus persistants |
| `ASSOC_DIST` | 1.5 m | Distance 3D max pour associer une mesure à un tracker | ↑ = moins de nouveaux trackers créés par erreur |
| `OUTLIER_STD` | 0.5 m | Seuil rejet outliers LiDAR (± médiane) | ↓ = filtrage plus agressif |
| `R_std` | 0.20 m | Bruit de mesure Kalman | ↑ = Kalman fait moins confiance aux mesures |
| `Q_std` | 0.02 m | Bruit de processus Kalman | Laisser faible (cône statique) |

---

## `cone_mapper_lidar.py` — SLAM LiDAR

| Paramètre | Valeur | Description | Impact si modifié |
|---|---|---|---|
| `MERGE_DIST` | 1.5 m | Distance max pour fusionner deux détections | ↑ = carte plus agressive, ↓ = cônes dupliqués |
| `COUNT_CAP` | 20 | Fenêtre de lissage (moyenne pondérée) | ↑ = stabilisation plus lente mais meilleure |
| `MIN_COUNT` | 3 | Confiance minimale pour afficher un cône | ↑ = moins de faux positifs sur la carte |
| `MAX_DEPTH_STRAIGHT` | 8.0 m | Portée LiDAR acceptée (ligne droite) | ↑ = carte plus anticipée |
| `MAX_DEPTH_TURN` | 5.0 m | Portée LiDAR réduite en virage | ↓ = virage plus prudent |
| `TURN_YAW_RATE` | 0.3 rad/s | Seuil de détection virage | ↓ = plus de virages détectés |
| `LINE_STRIDE` | 1 | Cônes utilisés comme points de contrôle | ↑ = courbe plus lisse (moins de points) |
| `MAX_LINE_STEP` | 8.0 m | Gap max entre deux cônes d'une ligne | ↓ = lignes segmentées plus tôt |
| `PUBLISH_HZ` | 2.0 | Fréquence de publication de la carte | ↑ = RViz plus réactif |

---

## `cone_mapper.py` — SLAM Stéréo

| Paramètre | Valeur | Différence vs LiDAR |
|---|---|---|
| `MAX_DEPTH_STRAIGHT` | 4.0 m | Stéréo moins fiable au-delà de 4 m |
| `MAX_DEPTH_TURN` | 4.0 m | Idem |
| `LINE_STRIDE` | 3 | Plus grand (stéréo moins précis → lissage fort) |
| `MAX_LINE_STEP` | 7.0 m | Légèrement moins (piste FS ≈ 5 m + 40% marge) |

---

## `yolo_stereo.py` — Vision Stéréo

| Paramètre | Valeur | Description |
|---|---|---|
| `BASELINE_M` | 0.64 m | Écartement entre cam1 et cam2 (settings.json FSDS) |
| `FOCAL_PX` | 208 px | Focale initiale (mise à jour depuis `/camera_info`) |
| `FSG_CONE_REAL_HEIGHT_M` | 0.325 m | Hauteur réelle d'un cône Formula Student |
| `Z_STEREO_MIN_M` | 2.0 m | Profondeur min stéréo fiable |
| `Z_STEREO_MAX_M` | 12.0 m | Profondeur max stéréo fiable |
| `CY_TOLERANCE_PX` | 15 px | Tolérance contrainte épipolaire |
| `DISP_MIN_BBOX_PX` | 8 px | Disparité min (→ `Z_max ≈ 16.6 m`) |
| `DISP_MAX_BBOX_PX` | 100 px | Disparité max (→ `Z_min ≈ 1.3 m`) |
| `DEPTH_OFFSET` | `{0: -1.5, 1: -2.0, 2: 0.0}` | Correction biais par classe (m) |

---

## `lidar_cluster.py` — Clustering DBSCAN

| Paramètre | Valeur | Description |
|---|---|---|
| `DBSCAN_EPS` | 0.5 m | Rayon de regroupement (diamètre cône ≈ 0.28 m) |
| `DBSCAN_MIN_SAMPLES` | 3 pts | Points min pour former un cluster valide |
| `MAX_CONE_DIST_M` | 15.0 m | Distance horizontale max acceptée |
| `MAX_CLUSTER_HEIGHT_M` | 0.6 m | Hauteur max d'un cluster (cône FS = 0.325 m + marge) |
| `MARKER_SCALE` | 0.4 m | Taille des sphères RViz |
| `MARKER_LIFETIME_NS` | 200 000 000 ns | Durée de vie markers RViz (0.2 s) |

---

## `cone_evaluator.py` — Évaluateur

| Paramètre | Valeur | Description |
|---|---|---|
| `--map` | `/slam_lidar/cone_map` | Topic de la carte SLAM à évaluer |
| Seuil fantôme | 3.0 m | Distance max pour être apparié à un cône GT |

---

## `eval_dashboard.py` — Dashboard précision

| Paramètre | Valeur | Description |
|---|---|---|
| `MAX_ERROR_M` | 1.5 m | Erreur de référence = 0 % de précision |
| `HISTORY_LEN` | 120 pts | Longueur de l'historique sparkline |

---

## Géométrie de piste Formula Student

| Dimension | Valeur |
|---|---|
| Largeur de piste | 6 m |
| Espacement cônes même côté | ~5 m (moins en virage) |
| Nombre de tours max | 10 |
| Hauteur cône FSG | 0.325 m |
| Diamètre cône FSG | 0.28 m |

Ces valeurs justifient les paramètres `MAX_LINE_STEP` (~5 m × 1.4 = 7–8 m) et `DBSCAN_EPS` (~0.28/2 + marge = 0.5 m).
