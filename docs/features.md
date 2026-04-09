# Fonctionnalités

## 1. Détection de cônes — YOLO Stéréo

**Fichier :** `python_stack/yolo_stereo.py`

Pipeline de détection visuelle par caméra stéréo :

- Inférence **YOLOv8** simultanée sur les deux flux caméra (batch gauche + droite)
- Modèle : `weights/best_FINAL.pt` — 3 classes : **JAUNE (0), BLEU (1), ORANGE (2)**

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

**Fichier :** `python_stack/yolo_lidar.py`

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

**Fichier :** `python_stack/lidar_cluster.py`

Pipeline sans YOLO, uniquement LiDAR :

- Filtrage sol : réception depuis `/lidar/obstacles` (produit par `lidar_ros.py`)
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

**Fichier :** `python_stack/cone_mapper_lidar.py`

| Paramètre | Valeur |
|---|---|
| Source | `/perception/lidar_detections` |
| Repère | `fsds/Lidar1` → `fsds/map` |
| Max profondeur (ligne droite) | 8 m |
| Max profondeur (virage) | 5 m |
| Distance de fusion | 1.5 m |
| Confiance minimale | 3 détections |

### SLAM Stéréo

**Fichier :** `python_stack/cone_mapper.py`

Même fonctionnement que le SLAM LiDAR mais avec une portée réduite (4 m) et un stride de 3 pour le lissage des lignes.

### Fonctionnalités communes

- **Suspension en virage** : aucun cône ajouté si `|yaw_rate| ≥ 0.3 rad/s`
- **Fusion de détections** : moyenne pondérée sur les 20 dernières mesures
- **Lignes de piste Catmull-Rom** : cônes jaunes (droite) et bleus (gauche)
- Publication avec **QoS latché** (TRANSIENT_LOCAL) → RViz reçoit la carte au démarrage

---

## 5. Gestion du repère TF

**Fichier :** `python_stack/odom_tf_publisher.py`

!!! warning "Nœud indispensable"
    Sans ce nœud, `fsds/map` n'existe pas dans l'arbre TF et tous les SLAM échouent.

- Souscrit à `/testing_only/odom`
- Publie le TF dynamique `fsds/map → fsds/FSCar` (~50 Hz)
- Publie le tracé `/slam/car_path` pour RViz

---

## 6. Évaluation de la précision

**Fichier :** `python_stack/cone_evaluator.py`

Compare les cônes SLAM avec la vérité terrain FSDS :

- Souscrit à `/testing_only/track` (`fs_msgs/Track`) — positions réelles des cônes
- Association nearest-neighbor (seuil 3 m)
- **Métriques** : erreur moy, RMSE, min, max, fantômes

**Publications :**

- `/evaluation/stats` — métriques texte (lu par `eval_dashboard.py`)
- `/evaluation/markers` — flèches d'erreur dans RViz

---

## 7. Dashboard d'évaluation

**Fichier :** `python_stack/eval_dashboard.py`

Interface graphique tkinter :

- **Jauge circulaire** : 0 % (erreur ≥ 1.5 m) → 100 % (erreur = 0 m)
- Code couleur : 🟢 >70 % / 🟡 40–70 % / 🔴 <40 %
- Historique sparkline sur 120 points
- ROS2 en thread daemon (non bloquant)

---

## 8. Monitoring des ressources

**Fichier :** `python_stack/system_monitor.py`

Monitore uniquement les processus de **perception embarquée** (FSDS exclu) :

=== "Catégories surveillées"
    | Catégorie | Processus |
    |---|---|
    | Scripts Python | `yolo_lidar`, `yolo_stereo`, `cone_mapper_lidar`, `cone_mapper`, `odom_tf_publisher`, `cone_evaluator` |
    | Middleware ROS2 | `ros2`, `fastrtps`, `fastdds`, `cyclonedds` |
    | Drivers capteurs | `velodyne`, `ouster`, `realsense`, `usb_cam`, `v4l2`, `zed` |

=== "Métriques affichées"
    - RAM en **Go** (RSS par processus)
    - CPU en **cœurs équivalents** (cpu_percent / 100)
    - VRAM en **Go** (via `nvidia-smi`)
    - **Max historique** depuis le démarrage
    - Sparklines 60 s

---

## 9. Contrôle manuel

**Fichier :** `python_stack/global_drive.py`

- Topic : `/control_command` (`fs_msgs/ControlCommand`)
- Clavier via **pynput** (thread séparé, non bloquant)
- Fréquence : 10 Hz

---

## 10. Launcher GUI

**Fichier :** `launcher.py`

- Liste tous les `.sh` du repo
- Double-clic pour lancer
- Panneau monitoring intégré (RAM, CPU, VRAM en temps réel)
- Bouton "Ouvrir dashboard complet" → lance `system_monitor.py`
