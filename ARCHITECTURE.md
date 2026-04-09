# IMT Driverless Stack — Architecture détaillée

> Ce document décrit en détail l'architecture logicielle, les flux de données,
> les topics ROS2 et les repères de coordonnées utilisés dans la stack.
> Objectif : permettre à la prochaine équipe de comprendre et d'étendre le système.

---

## Vue d'ensemble

La stack est organisée en **deux pipelines parallèles** (stéréo et LiDAR) qui partagent la même infrastructure de localisation TF et peuvent fonctionner simultanément ou indépendamment.

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                          SIMULATEUR FSDS                                    │
│  Caméra stéréo (cam1/cam2) │ LiDAR 3D (Lidar1) │ Odométrie │ GPS │ IMU    │
└──────────────┬─────────────────────┬────────────────────┬───────────────────┘
               │ ROS2 Bridge          │                    │
               ▼                     ▼                    ▼
    ┌──────────────────┐   ┌──────────────────┐   ┌───────────────────┐
    │   yolo_stereo.py │   │   yolo_lidar.py  │   │ odom_tf_publisher │
    │ (YOLO + stéréo)  │   │ (YOLO + Kalman)  │   │   (TF fsds/map)   │
    └────────┬─────────┘   └────────┬─────────┘   └───────────────────┘
             │                      │
    /perception/                /perception/
    stereo_detections           lidar_detections
             │                      │
             ▼                      ▼
    ┌──────────────────┐   ┌──────────────────────┐
    │  cone_mapper.py  │   │ cone_mapper_lidar.py  │
    │  (SLAM stéréo)   │   │   (SLAM LiDAR)        │
    └────────┬─────────┘   └──────────┬────────────┘
             │                        │
    /slam/cone_map          /slam_lidar/cone_map
    /slam/stats             /slam_lidar/stats
             │                        │
             └───────────┬────────────┘
                         ▼
              ┌────────────────────┐       ┌──────────────────┐
              │  cone_evaluator.py │  ←──  │ /testing_only/   │
              │   (comparaison GT) │       │      track       │
              └────────┬───────────┘       └──────────────────┘
                       │
              /evaluation/stats
              /evaluation/markers
                       │
                       ▼
              ┌──────────────────┐
              │ eval_dashboard.py│
              │  (GUI tkinter)   │
              └──────────────────┘
```

---

## Repères de coordonnées (TF Frames)

### Arbre TF complet

```
fsds/map  (repère monde fixe — créé par odom_tf_publisher.py)
    └── fsds/FSCar  (repère voiture — dynamique, mis à jour à chaque odom)
            ├── fsds/cam1    (caméra gauche — TF statique, publié par le bridge)
            ├── fsds/cam2    (caméra droite — TF statique, publié par le bridge)
            └── fsds/Lidar1  (LiDAR — TF statique, publié par le bridge)
```

### Conventions d'axes

| Frame | X | Y | Z | Notes |
|---|---|---|---|---|
| `fsds/map` | Est (avant au départ) | Gauche | Haut | Repère monde NED-like |
| `fsds/FSCar` | Avant voiture | Gauche | Haut | Suit la pose de la voiture |
| `fsds/cam1` | Avant (profondeur optique) | Gauche | Haut | Convention physique FSDS |
| `fsds/Lidar1` | Avant | Gauche | Haut | Même orientation que voiture |

> **IMPORTANT** : Le bridge FSDS ne publie **pas** le TF `fsds/map`.
> Ce repère est créé **uniquement** par `odom_tf_publisher.py`.
> Sans ce nœud, tous les `tf2_ros.Buffer.transform()` vers `fsds/map` échouent.

---

## Topics ROS2 — Tableau complet

### Topics publiés par le bridge FSDS

| Topic | Type | Description | Frame |
|---|---|---|---|
| `/fsds/cam1/image_color` | `sensor_msgs/Image` | Flux caméra gauche (640×480, RGB8) | `fsds/cam1` |
| `/fsds/cam2/image_color` | `sensor_msgs/Image` | Flux caméra droite (640×480, RGB8) | `fsds/cam2` |
| `/fsds/cam1/camera_info` | `sensor_msgs/CameraInfo` | Paramètres intrinsèques cam1 (focale, cx, cy) | `fsds/cam1` |
| `/fsds/cam2/camera_info` | `sensor_msgs/CameraInfo` | Paramètres intrinsèques cam2 | `fsds/cam2` |
| `/lidar/Lidar1` | `sensor_msgs/PointCloud2` | Nuage de points LiDAR 3D (~100k pts/frame) | `fsds/Lidar1` |
| `/testing_only/odom` | `nav_msgs/Odometry` | Odométrie ground truth (position + orientation + vitesses) | `fsds/map` |
| `/testing_only/track` | `fs_msgs/Track` | Positions réelles de tous les cônes de la piste | `fsds/map` |
| `/imu/Imu1` | `sensor_msgs/Imu` | Accéléromètre + gyroscope | `fsds/FSCar` |

### Topics publiés par `odom_tf_publisher.py`

| Topic | Type | Description |
|---|---|---|
| `/slam/car_path` | `nav_msgs/Path` | Historique complet de la trajectoire voiture (pour RViz) |
| *(TF)* | `tf2_msgs/TFMessage` | Transform `fsds/map → fsds/FSCar` (dynamique, ~50 Hz) |

### Topics publiés par `yolo_stereo.py`

| Topic | Type | Description |
|---|---|---|
| `/perception/stereo_detections` | `vision_msgs/Detection2DArray` | Cônes détectés avec position 3D (dans `fsds/cam1`) |
| `/yolo_stereo/debug_image` | `sensor_msgs/Image` | Image annotée avec bbox YOLO et profondeur estimée |

Structure d'un message `Detection2DArray` :
```
header.frame_id = 'fsds/cam1'
results[i]:
  ObjectHypothesisWithPose:
    hypothesis.class_id = "0"/"1"/"2"  (JAUNE/BLEU/ORANGE)
    hypothesis.score    = confidence YOLO
    pose.pose.position.x = position X dans fsds/cam1 (avant)
    pose.pose.position.y = position Y dans fsds/cam1 (gauche)
    pose.pose.position.z = profondeur Z estimée (metres)
```

### Topics publiés par `yolo_lidar.py`

| Topic | Type | Description |
|---|---|---|
| `/perception/lidar_detections` | `vision_msgs/Detection2DArray` | Cônes confirmés avec position 3D Kalman (dans `fsds/Lidar1`) |
| `/yolo_lidar/debug_image` | `sensor_msgs/Image` | Image avec bbox + points LiDAR projetés |
| `/yolo_lidar/cone_markers` | `visualization_msgs/MarkerArray` | Marqueurs 3D des cônes trackés (repère LiDAR) |

### Topics publiés par `cone_mapper.py` (SLAM stéréo)

| Topic | Type | QoS | Description |
|---|---|---|---|
| `/slam/cone_map` | `visualization_msgs/MarkerArray` | Latché (TRANSIENT_LOCAL) | Carte globale des cônes dans `fsds/map` |
| `/slam/stats` | `std_msgs/String` | Best effort | Statistiques texte : `N cônes | Y JAUNE | B BLEU` |

Structure du `MarkerArray` `/slam/cone_map` :
```
markers[]:
  ns = 'cone_map'         → cônes cylindriques
  ns = 'cone_labels'      → texte compteur sur chaque cône
  ns = 'yellow_line'      → ligne Catmull-Rom côté droit
  ns = 'blue_line'        → ligne Catmull-Rom côté gauche
  type = CYLINDER         → cônes (scale.x=scale.y=0.28, scale.z=0.33)
  type = TEXT_VIEW_FACING → labels
  type = LINE_STRIP        → lignes de piste
  color: jaune=(1,1,0), bleu=(0,0.2,1), orange=(1,0.4,0)
  pose.position: position dans fsds/map
  id: identifiant unique (ordre de découverte = ordre de passage)
```

### Topics publiés par `cone_mapper_lidar.py` (SLAM LiDAR)

| Topic | Type | QoS | Description |
|---|---|---|---|
| `/slam_lidar/cone_map` | `visualization_msgs/MarkerArray` | Latché | Carte globale LiDAR dans `fsds/map` |
| `/slam_lidar/stats` | `std_msgs/String` | Best effort | Statistiques SLAM LiDAR |

### Topics publiés par `cone_evaluator.py`

| Topic | Type | QoS | Description |
|---|---|---|---|
| `/evaluation/stats` | `std_msgs/String` | Best effort (10 Hz) | Métriques texte parsées par `eval_dashboard.py` |
| `/evaluation/markers` | `visualization_msgs/MarkerArray` | Latché | Flèches d'erreur + labels dans RViz |

Format du message `/evaluation/stats` :
```
"Comparaison 42/45 cônes | moy=0.123m | RMSE=0.156m | min=0.012m max=0.487m | fantômes=3"
```

### Topics publiés par `global_drive.py`

| Topic | Type | Description |
|---|---|---|
| `/control_command` | `fs_msgs/ControlCommand` | Commandes de conduite (throttle, brake, steering) |

---

## Pipeline détaillé — YOLO LiDAR + SLAM LiDAR

C'est le pipeline **principal recommandé** (le plus précis).

### Étape 1 : Synchronisation temporelle des capteurs

`yolo_lidar.py` utilise un `message_filters.ApproximateTimeSynchronizer` avec :
- `/fsds/cam1/image_color` (caméra)
- `/lidar/Lidar1` (LiDAR)
- Tolérance de synchronisation : `slop = 0.1 s`

### Étape 2 : Détection YOLO

```
image → YOLO (GPU) → list[(bbox, class_id, score)]
```
- Modèle chargé une seule fois au démarrage (`torch.device('cuda')` si disponible)
- Résultats filtrés : `score > 0` (pas de seuil de confiance explicite)

### Étape 3 : Projection LiDAR dans l'image

Pour chaque point `(X, Y, Z)` dans le repère `fsds/Lidar1` :
1. Transformation vers `fsds/cam1` via TF2 : `tf_buffer.transform(point_stamped, 'fsds/cam1')`
2. Projection dans le plan image via `PinholeCameraModel.project3dToPixel(x, y, z)`
3. Filtrage : garder uniquement les points dont le pixel tombe dans une bounding box YOLO
4. Filtrage de profondeur : `z > 0` (en face de la caméra)

### Étape 4 : Rejet des outliers LiDAR

Pour chaque groupe de points dans une bbox :
- Calcul de la **distance horizontale** médiane : `d = sqrt(x²+y²)` dans le repère LiDAR
- Rejet des points dont `|d - médiane| > OUTLIER_STD = 0.5 m`
- Position estimée = **centroïde pondéré** des points restants (poids = inverse de la distance)

### Étape 5 : Filtre de Kalman par cône

```python
# Pour chaque détection (x, y, z) dans fsds/Lidar1 :
tracker = ConeTracker.associate(x, y, z, max_dist=ASSOC_DIST=1.5m)
if tracker:
    tracker.kalman.update(x, y, z)   # mise à jour Kalman
    tracker.hits += 1
else:
    new_tracker = ConeTracker(x, y, z)  # nouveau tracker

# Publication si hits >= MIN_HITS = 3
```

Le **filtre de Kalman** est à état `[px, py, pz]` statique :
- `F = I` : pas de prédiction (le cône ne bouge pas)
- `H = I` : on mesure directement la position
- `Q = 0.02² × I` : bruit processus très faible (cône fixe)
- `R = 0.20² × I` : bruit mesure LiDAR (~20 cm)
- Gain de Kalman `K = P(P+R)⁻¹` converge vers 0 après ~10 mesures

### Étape 6 : Publication `/perception/lidar_detections`

Chaque cône confirmé est publié avec :
- `pose.pose.position` = position Kalman dans `fsds/Lidar1`
- `hypothesis.class_id` = couleur YOLO (`"0"`, `"1"`, `"2"`)
- `hypothesis.score` = confiance YOLO

---

## Pipeline détaillé — SLAM LiDAR (`cone_mapper_lidar.py`)

### Étape 1 : Réception et filtrage par profondeur

Pour chaque détection reçue sur `/perception/lidar_detections` :
```python
depth = sqrt(x² + y²)   # distance horizontale dans fsds/Lidar1
if yaw_rate < TURN_YAW_RATE:
    max_depth = MAX_DEPTH_STRAIGHT = 8.0 m
else:
    max_depth = MAX_DEPTH_TURN = 5.0 m
if depth > max_depth:
    continue  # ignoré
```

### Étape 2 : Transformation dans fsds/map

```python
pt_lidar = PointStamped()
pt_lidar.header.frame_id = 'fsds/Lidar1'
pt_lidar.point = (x, y, z)
pt_map = tf_buffer.transform(pt_lidar, 'fsds/map')
```

### Étape 3 : Fusion des cônes (Merge + moyenne pondérée)

Pour chaque position `(x_map, y_map)` transformée dans `fsds/map` :
1. Chercher un cône existant à moins de `MERGE_DIST = 1.5 m`
2. Si trouvé : **moyenne pondérée** sur les `COUNT_CAP = 20` dernières mesures
   ```
   cone.x = (cone.x × min(cone.count, COUNT_CAP) + x_new) / (min(cone.count, COUNT_CAP) + 1)
   ```
3. Si non trouvé : créer un nouveau cône avec `count = 1`
4. Cône publié uniquement si `count >= MIN_COUNT = 3`

### Étape 4 : Détection virage (anti-pollution de carte)

```python
# Depuis /testing_only/odom → twist.angular.z
yaw_rate = odom.twist.twist.angular.z
if abs(yaw_rate) >= TURN_YAW_RATE:
    return  # aucun cône ajouté en virage
```
En virage, les positions LiDAR sont moins fiables car :
- La voiture tourne → le repère LiDAR pivote → la projection TF peut être décalée si l'odométrie est en retard

### Étape 5 : Tracé des lignes de piste (Catmull-Rom)

```python
# Tri des cônes par ID (ordre de découverte = ordre de passage voiture)
yellow_cones = sorted([c for c in cones if c.color == 0], key=lambda c: c.id)
blue_cones   = sorted([c for c in cones if c.color == 1], key=lambda c: c.id)

# Pour chaque liste, on segmente si gap > MAX_LINE_STEP = 8 m
# Puis on interpole avec des splines Catmull-Rom
# Points de contrôle : 1 cône sur LINE_STRIDE=1 (LiDAR) ou 3 (stéréo)
```

La spline **Catmull-Rom** est calculée avec des points fantômes aux extrémités pour éviter les effets de bord. Elle produit une courbe passant exactement par les points de contrôle.

### Étape 6 : Publication périodique (2 Hz)

`/slam_lidar/cone_map` est publié avec QoS **TRANSIENT_LOCAL** (latché) :
- Un nouveau souscripteur reçoit automatiquement le dernier message même s'il s'abonne après la publication
- Cela permet à RViz d'afficher la carte dès qu'il démarre, sans attendre la prochaine publication

---

## Pipeline détaillé — YOLO Stéréo + SLAM Stéréo

### Estimation de profondeur stéréo

```
cam1 (gauche) → bbox list_gauche
cam2 (droite) → bbox list_droite

Pour chaque cône dans list_gauche :
  Chercher correspondant dans list_droite tel que :
    - même classe (couleur) OU classe orange tolérée
    - |cy_gauche - cy_droite| < CY_TOLERANCE_PX = 15  (contrainte épipolaire)
    - cx_gauche > cx_droite  (objet plus à droite sur cam gauche)
    - DISP_MIN = 8 px < (cx_gauche - cx_droite) < DISP_MAX = 100 px

  Si appariement valide :
    Z = FOCAL_PX × BASELINE_M / (cx_gauche - cx_droite)
       = 208 × 0.64 / disparité

  Sinon (fallback monoculaire) :
    Z = FOCAL_PX × FSG_CONE_REAL_HEIGHT_M / h_bbox_pixels
       = 208 × 0.325 / hauteur_bbox
```

### Correction HSV orange

Après estimation de profondeur :
- Extraire la ROI de l'image correspondant à la bbox
- Convertir en HSV
- Si le ratio de pixels orange (H ∈ [10, 25]) > seuil → reclasser comme ORANGE (2)
- Évite les confusions YOLO jaune/orange fréquentes dans FSDS

---

## Communication entre processus — Schéma temporel

```
t=0        FSDS publie : /fsds/cam1/image_color (30 Hz)
                         /lidar/Lidar1 (10 Hz)
                         /testing_only/odom (50 Hz)

t=0        odom_tf_publisher reçoit odom → publie TF fsds/map→fsds/FSCar (50 Hz)

t=0.033    yolo_lidar reçoit image + LiDAR (synchronisé) →
           YOLO inference (~50 ms GPU) →
           projection LiDAR →
           Kalman update →
           publie /perception/lidar_detections (~10 Hz)

t=0.083    cone_mapper_lidar reçoit /perception/lidar_detections →
           transform TF (fsds/Lidar1 → fsds/map) →
           merge cônes →
           (toutes les 0.5s) publie /slam_lidar/cone_map

           cone_evaluator reçoit /slam_lidar/cone_map →
           compare avec gt_cones →
           publie /evaluation/stats (10 Hz)

           eval_dashboard reçoit /evaluation/stats →
           met à jour GUI tkinter (1 Hz)
```

---

## Workspace ROS2 (`ros_workspace/`)

Le workspace contient le package `fs_msgs` compilé localement.
Ce package définit les messages spécifiques FSDS :

| Message | Champs | Utilisé par |
|---|---|---|
| `fs_msgs/Track` | `track: Cone[]` | `/testing_only/track` (GT cônes) |
| `fs_msgs/Cone` | `location: Point`, `color: uint8` | `cone_evaluator.py` |
| `fs_msgs/ControlCommand` | `throttle`, `brake`, `steering: float32` | `global_drive.py` |

Constantes `fs_msgs/Cone.color` :
```
BLUE         = 0
YELLOW       = 1
ORANGE_BIG   = 2
ORANGE_SMALL = 3
UNKNOWN      = 4
```

> **Note** : Cette convention est **inversée** par rapport aux classes YOLO du modèle
> (`0=JAUNE, 1=BLEU, 2=ORANGE`). La table de correspondance dans `cone_evaluator.py` :
> `FS_COLOR = {0: 1, 1: 0, 2: 2, 3: 2, 4: -1}`

---

## Compatibilité ROS2 Galactic / Iron

Deux points de compatibilité sont gérés dans le code :

### 1. Import `point_cloud2`
```python
try:
    import sensor_msgs_py.point_cloud2 as pc2   # Iron / Humble
except ImportError:
    from sensor_msgs import point_cloud2 as pc2  # Galactic
```

### 2. Détection automatique de la distro dans les scripts `.sh`
```bash
if   [ -f "/opt/ros/iron/setup.bash" ];    then MY_ROS_DISTRO="iron"
elif [ -f "/opt/ros/galactic/setup.bash" ]; then MY_ROS_DISTRO="galactic"
elif [ -f "/opt/ros/humble/setup.bash" ];   then MY_ROS_DISTRO="humble"
fi
```

---

## Paramètres critiques à connaître

### `yolo_lidar.py`
| Paramètre | Valeur | Impact |
|---|---|---|
| `MIN_HITS` | 3 | Nombre de détections avant confirmation d'un cône |
| `MAX_MISSES` | 8 | Frames sans détection avant suppression |
| `ASSOC_DIST` | 1.5 m | Distance max pour associer une mesure à un tracker |
| `OUTLIER_STD` | 0.5 m | Seuil rejet outliers LiDAR |
| `R_std` | 0.20 m | Bruit mesure Kalman |

### `cone_mapper_lidar.py`
| Paramètre | Valeur | Impact |
|---|---|---|
| `MERGE_DIST` | 1.5 m | Distance de fusion de deux cônes → réduire si carte bruitée |
| `COUNT_CAP` | 20 | Fenêtre de lissage de position → augmenter pour plus de stabilité |
| `MIN_COUNT` | 3 | Confiance minimale pour afficher un cône |
| `MAX_DEPTH_STRAIGHT` | 8.0 m | Portée LiDAR acceptée en ligne droite |
| `MAX_DEPTH_TURN` | 5.0 m | Portée réduite en virage |
| `TURN_YAW_RATE` | 0.3 rad/s | Seuil de détection virage |
| `LINE_STRIDE` | 1 | Points de contrôle Catmull-Rom (1 = tous les cônes) |
| `MAX_LINE_STEP` | 8.0 m | Gap max entre deux cônes d'une même ligne |

### `yolo_stereo.py`
| Paramètre | Valeur | Impact |
|---|---|---|
| `BASELINE_M` | 0.64 m | Écartement des caméras (défini dans `settings.json` FSDS) |
| `FOCAL_PX` | 208 px | Focale estimée (mise à jour depuis `camera_info`) |
| `Z_STEREO_MIN_M` | 2.0 m | En dessous : fallback mono |
| `Z_STEREO_MAX_M` | 12.0 m | Au-dessus : fallback mono |
| `CY_TOLERANCE_PX` | 15 px | Tolérance contrainte épipolaire |

---

## Déploiement sur véhicule réel

Pour passer du simulateur au véhicule réel, remplacer les topics FSDS par les drivers capteurs réels :

| Topic FSDS | Équivalent réel | Driver |
|---|---|---|
| `/fsds/cam1/image_color` | `/camera/left/image_raw` | `usb_cam`, `realsense2` |
| `/lidar/Lidar1` | `/velodyne_points` ou `/ouster/points` | `velodyne_driver`, `ouster_driver` |
| `/testing_only/odom` | `/odom` ou `/gps/odom` | `robot_localization`, `nmea_navsat` |
| `/testing_only/track` | **N/A** (uniquement en simulation) | — |
| *(TF statiques capteurs)* | Fichier URDF + `robot_state_publisher` | — |

> Le node `cone_evaluator.py` ne fonctionnera **pas** sur véhicule réel (pas de ground truth).
> Les nodes `yolo_lidar.py`, `cone_mapper_lidar.py`, `odom_tf_publisher.py` sont compatibles
> avec le matériel réel sans modification (adapter uniquement les noms de topics).

---

## Structure des fichiers

```
IMT-Driverless-Stack/
├── launcher.py                    # GUI de lancement centralisé
├── CLAUDE.md                      # Référence rapide pour l'IA
├── FEATURES.md                    # Description des fonctionnalités
├── ARCHITECTURE.md                # Ce fichier
├── SETUP_IMT_V1.sh               # Installation depuis zéro
├── setup_update.sh                # Mise à jour intelligente
├── auto_launch_FSDS.sh            # Lancement rapide FSDS seul
│
├── python_stack/
│   ├── weights/
│   │   └── best_FINAL.pt          # Modèle YOLO entraîné (cônes FS)
│   │
│   ├── yolo_stereo.py             # Pipeline caméra stéréo
│   ├── yolo_lidar.py              # Pipeline LiDAR + Kalman
│   ├── cone_mapper.py             # SLAM stéréo
│   ├── cone_mapper_lidar.py       # SLAM LiDAR (recommandé)
│   ├── odom_tf_publisher.py       # Indispensable : TF fsds/map
│   ├── cone_evaluator.py          # Évaluation vs ground truth
│   ├── eval_dashboard.py          # GUI précision SLAM
│   ├── system_monitor.py          # GUI ressources système
│   ├── global_drive.py            # Contrôle clavier
│   │
│   ├── slam_lidar.sh              # Lancement complet LiDAR (recommandé)
│   ├── slam_stereo.sh             # Lancement complet stéréo
│   ├── auto_eval.sh               # Évaluateur seul
│   └── auto_yolo_lidar.sh         # YOLO LiDAR sans SLAM
│
└── ros_workspace/
    └── src/fs_msgs/               # Messages ROS2 FSDS compilés localement
```
