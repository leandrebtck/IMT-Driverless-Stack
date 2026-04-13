# Fonctionnalités

## 1. Détection de cônes — YOLO Stéréo

**Fichier :** `python_scripts/1perception/yolo_stereo.py`

Pipeline de détection visuelle par caméra stéréo :

- Inférence **YOLOv8** simultanée sur les deux flux caméra (batch gauche + droite)
- Modèle : `python_scripts/1perception/weights/best_FINAL.pt` — 3 classes : **JAUNE (0), BLEU (1), ORANGE (2)**

### Estimation de profondeur stéréo

$$Z = \frac{f \times B}{cx_{gauche} - cx_{droite}}$$

avec `f = 208 px`, `B = 0.64 m` (baseline stéréo)

| Paramètre | Valeur | Description |
|---|---|---|
| Plage stéréo fiable | 2–12 m | En dehors → fallback monoculaire |
| Tolérance épipolaire | ±15 px | Contrainte verticale |
| Disparité min/max | 8–100 px | Filtrage des mauvais appariements |

- **Fallback monoculaire** : estimation par taille apparente (`h_réel = 0.325 m`)
- **Correction HSV** : reclassification orange/jaune par analyse couleur
- **Correction de biais** : `-1.5 m` jaune, `-2.0 m` bleu (biais simulateur)

**Publication :** `/perception/stereo_detections` (`vision_msgs/Detection2DArray`)

---

## 2. Détection de cônes — YOLO + LiDAR

**Fichier :** `python_scripts/1perception/yolo_lidar.py`

Fusion caméra + LiDAR avec filtre de Kalman par cône :

1. Inférence YOLO sur caméra gauche
2. Projection des points LiDAR dans l'image via `PinholeCameraModel` + TF2
3. Association bbox ↔ points LiDAR
4. Rejet outliers : `|distance - médiane| > 0.5 m`
5. Position 3D par centroïde pondéré
6. **Filtre de Kalman** par cône (état `[px, py, pz]`, modèle statique)

### Filtre de Kalman

| Paramètre | Valeur | Description |
|---|---|---|
| `MIN_HITS` | 3 | Détections min avant confirmation |
| `MAX_MISSES` | 8 | Frames sans détection avant suppression |
| `ASSOC_DIST` | 1.5 m | Distance max association mesure→tracker |
| `R_std` | 0.20 m | Bruit mesure LiDAR |
| `Q_std` | 0.02 m | Bruit processus (cône statique) |

Convergence : ~5 mises à jour → précision **±5 cm**

**Publication :** `/perception/lidar_detections` (`vision_msgs/Detection2DArray`)

---

## 3. Détection LiDAR seule — DBSCAN

**Fichier :** `python_scripts/1perception/lidar_cluster.py`

Pipeline sans YOLO, uniquement LiDAR :

- Filtrage sol : réception depuis `/lidar/obstacles` (produit par `python_scripts/1perception/lidar_ros.py`)
- **Clustering DBSCAN** en 2D (projection XY vue du dessus)
- Validation géométrique : hauteur cluster < 0.6 m
- Score de confiance basé sur la distance (`1 - d/15 m`)
- `class_id = -1` (couleur inconnue) → affichage gris dans le mapper

| Paramètre | Valeur |
|---|---|
| `DBSCAN_EPS` | 0.5 m |
| `DBSCAN_MIN_SAMPLES` | 3 pts |
| `MAX_CONE_DIST_M` | 15 m |
| `MAX_CLUSTER_HEIGHT_M` | 0.6 m |

**Publications :** `/lidar/cone_markers` (RViz) + `/perception/lidar_detections`

---

## 4. SLAM — Cartographie globale des cônes

### SLAM LiDAR *(recommandé)*

**Fichier :** `python_scripts/2slam/cone_mapper_lidar.py`

| Paramètre | Valeur |
|---|---|
| Source | `/perception/lidar_detections` |
| Repère | `fsds/Lidar1` → `fsds/map` |
| Max profondeur (ligne droite) | 8 m |
| Max profondeur (virage) | 5 m |
| Distance de fusion | 1.5 m |
| Confiance minimale | 3 détections |

### SLAM Stéréo

**Fichier :** `python_scripts/2slam/cone_mapper.py`

Même fonctionnement que le SLAM LiDAR mais avec une portée réduite (4 m) et un stride de 3 pour le lissage des lignes.

### SLAM Skidpad

**Fichiers :** `python_scripts/2slam/cone_mapper_skidpad_lidar.py`, `python_scripts/2slam/cone_mapper_skidpad_stereo.py`

Variantes dédiées à l'épreuve Skidpad :

- Pas de blocage en virage (le Skidpad est entièrement composé de virages)
- Affichage centerline : **trait vert jaune↔bleu** + sphère blanche au midpoint
- Pas de lignes latérales (inutiles sur cercles)
- Topics : `/slam_skidpad_lidar/cone_map`, `/slam_skidpad_stereo/cone_map`

### Fonctionnalités communes

- **Suspension en virage** : aucun cône ajouté si `|yaw_rate| ≥ 0.3 rad/s` *(circuit standard uniquement)*
- **Fusion de détections** : moyenne pondérée sur les 20 dernières mesures
- **Lignes de piste Catmull-Rom** : cônes jaunes (droite) et bleus (gauche) *(circuit standard)*
- Publication avec **QoS latché** (TRANSIENT_LOCAL) → RViz reçoit la carte au démarrage

---

## 5. Pilotage autonome — Circuit

**Fichier :** `python_scripts/3control/centerline_follower.py`

Suit le milieu de la piste par Pure Pursuit sur les midpoints SLAM :

- Construit un chemin de waypoints jaune↔bleu depuis la carte complète
- Ordonnancement nearest-neighbor depuis la position de la voiture
- Fallback local si la carte est insuffisante (1er tour)
- Correction virage serré : freinage si `|steering| > CORNER_STEER` et `speed > 0.4 m/s`

```bash
python3 python_scripts/3control/centerline_follower.py --map /slam_lidar/cone_map
```

---

## 6. Pilotage autonome — Skidpad

**Fichier :** `python_scripts/3control/skidpad_driver.py`

Pilotage géométrique pur (carte connue) :

| État | Description |
|---|---|
| `STRAIGHT` | Avance en ligne droite jusqu'à `STRAIGHT_DIST` |
| `RIGHT_1/2` | Pure Pursuit sur cercle droit (sens horaire), 2 tours |
| `CROSS` | Pure Pursuit vers le cercle gauche |
| `LEFT_1/2` | Pure Pursuit sur cercle gauche (sens antihoraire), 2 tours |
| `EXIT_STR` | Ligne droite de sortie jusqu'aux cônes orange ou 25 m max |
| `EXIT` | Arrêt complet |

Centres des cercles calculés **analytiquement** depuis la position et le cap à la fin de la ligne droite (aucun SLAM nécessaire). Comptage de tours par intégration de l'angle signé (direction du cercle uniquement, oscillations ignorées).

```bash
python3 python_scripts/3control/skidpad_driver.py --map /slam_skidpad_lidar/cone_map --straight 13.0
```

---

## 7. Pilotage autonome — Accélération

**Fichier :** `python_scripts/3control/acceleration_driver.py`

Épreuve ligne droite 75 m :

| État | Déclencheur | Commande |
|---|---|---|
| `ACCEL` | Départ | Plein gaz, correction de cap (P sur yaw) |
| `BRAKE` | `dist ≥ 65 m` ou cônes orange à `< 8 m` (après 40 m) | Freinage maximal |
| `STOP` | `vitesse < 0.3 m/s` | Maintien frein |

```bash
python3 python_scripts/3control/acceleration_driver.py --dist 75.0 --brake 10.0
```

---

## 8. Gestion du repère TF

**Fichier :** `python_scripts/1perception/odom_tf_publisher.py`

!!! warning "Nœud indispensable"
    Sans ce nœud, `fsds/map` n'existe pas dans l'arbre TF et tous les SLAM échouent.

- Souscrit à `/testing_only/odom`
- Publie le TF dynamique `fsds/map → fsds/FSCar` (~50 Hz)
- Publie le tracé `/slam/car_path` pour RViz

---

## 9. Évaluation de la précision

**Fichier :** `python_scripts/1perception/cone_evaluator.py`

Compare les cônes SLAM avec la vérité terrain FSDS :

- Souscrit à `/testing_only/track` (`fs_msgs/Track`) — positions réelles des cônes
- Association nearest-neighbor (seuil 3 m)
- **Métriques** : erreur moy, RMSE, min, max, fantômes

**Publications :**

- `/evaluation/stats` — métriques texte (lu par `eval_dashboard.py`)
- `/evaluation/markers` — flèches d'erreur dans RViz

---

## 10. Dashboard d'évaluation

**Fichier :** `python_scripts/performance/eval_dashboard.py`

Interface graphique tkinter :

- **Jauge circulaire** : 0 % (erreur ≥ 1.5 m) → 100 % (erreur = 0 m)
- Code couleur : 🟢 >70 % / 🟡 40–70 % / 🔴 <40 %
- Historique sparkline sur 120 points
- ROS2 en thread daemon (non bloquant)

---

## 11. Monitoring des ressources

**Fichier :** `python_scripts/performance/system_monitor.py`

Monitore uniquement les processus de **perception embarquée** (FSDS exclu) :

=== "Catégories surveillées"
    | Catégorie | Processus |
    |---|---|
    | Scripts Python | `yolo_lidar`, `yolo_stereo`, `cone_mapper*`, `odom_tf_publisher`, `cone_evaluator` |
    | Middleware ROS2 | `ros2`, `fastrtps`, `fastdds`, `cyclonedds` |
    | Drivers capteurs | `velodyne`, `ouster`, `realsense`, `usb_cam`, `v4l2`, `zed` |

=== "Métriques affichées"
    - RAM en **Go** (RSS par processus)
    - CPU en **cœurs équivalents** (cpu_percent / 100)
    - VRAM en **Go** (via `nvidia-smi`)
    - **Max historique** depuis le démarrage
    - Sparklines 60 s

---

## 12. Contrôle manuel

**Fichier :** `python_scripts/3control/global_drive.py`

- Topic : `/control_command` (`fs_msgs/ControlCommand`)
- Clavier via **pynput** (thread séparé, non bloquant)
- Fréquence : 10 Hz

---

## 13. Launcher GUI

**Fichier :** `launcher.py`

- Liste tous les `.sh` du repo
- Double-clic pour lancer
- Panneau monitoring intégré (RAM, CPU, VRAM en temps réel)
- Bouton "Ouvrir dashboard complet" → lance `python_scripts/performance/system_monitor.py`
