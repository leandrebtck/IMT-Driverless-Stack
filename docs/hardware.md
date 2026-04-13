# Besoins matériels — Véhicule réel

Cette page liste les capteurs nécessaires pour faire tourner la stack sur un véhicule Formula Student physique, avec leurs spécifications minimales et les modèles connus compatibles.

---

## Vue d'ensemble

La stack repose sur **trois capteurs obligatoires** et une **unité de calcul embarquée** :

| Capteur | Pipeline concerné | Obligatoire |
|---|---|---|
| Caméra stéréo | YOLO stéréo, profondeur cône | Oui (pipeline stéréo) |
| LiDAR 3D | YOLO + LiDAR, SLAM LiDAR | Oui (pipeline LiDAR) |
| Odométrie (IMU + GPS/encodeurs) | TF dynamique, SLAM | Oui (tous pipelines) |
| GPU embarqué | Inférence YOLOv8 | Oui |

!!! note "Pipelines indépendants"
    Les pipelines stéréo et LiDAR peuvent fonctionner séparément. En compétition, le **pipeline LiDAR est recommandé** (meilleure précision SLAM). Il faut néanmoins la caméra gauche pour la détection YOLO des couleurs.

---

## 1. LiDAR 3D

### Rôle dans la stack

- `yolo_lidar.py` : fusion YOLO + LiDAR pour localiser les cônes en 3D
- `lidar_cluster.py` : détection de cônes par DBSCAN sans caméra
- `cone_mapper_lidar.py` : SLAM carte globale

### Spécifications requises

| Paramètre | Minimum | Recommandé |
|---|---|---|
| Nombre de plans verticaux | 16 | 32+ |
| Portée | 20 m | 30 m+ |
| Champ horizontal | 180° (avant) | 360° |
| Champ vertical | ±10° minimum | ±15°+ |
| Fréquence de rotation | 10 Hz | 10–20 Hz |
| Précision de distance | ±3 cm | ±2 cm |
| Format de sortie ROS2 | `sensor_msgs/PointCloud2` | — |

!!! warning "Densité de points critique"
    À 8 m de distance, un cône Formula Student (Ø 28 cm) doit être frappé par **au moins 3 points** pour que le DBSCAN le détecte (`DBSCAN_MIN_SAMPLES = 3`). Un LiDAR 8 plans peut être insuffisant au-delà de 6 m.

### Modèles compatibles testés / connus

| Modèle | Plans | Portée | Driver ROS2 | Compatibilité |
|---|---|---|---|---|
| Velodyne VLP-16 | 16 | 100 m | `velodyne_driver` | ✅ Validé simulateur |
| Velodyne VLP-32C | 32 | 200 m | `velodyne_driver` | ✅ |
| Ouster OS1-32 | 32 | 120 m | `ouster_ros` | ✅ |
| Ouster OS0-32 | 32 | 50 m | `ouster_ros` | ✅ (FOV vertical ±45°, adapté cône proche) |
| Livox Mid-360 | Non-répétitif | 70 m | `livox_ros_driver2` | ⚠️ Nécessite adaptation (format non standard) |
| RPLIDAR A3 | 1 (2D) | 25 m | `rplidar_ros` | ❌ 2D uniquement, incompatible `yolo_lidar.py` |

### Topic à configurer

```python
# Dans config.yaml
lidar_detections: "/perception/lidar_detections"

# Dans yolo_lidar.py — frame à adapter
LIDAR_FRAME = "velodyne"   # ou "os_sensor", etc.
```

---

## 2. Caméra stéréo

### Rôle dans la stack

- `yolo_lidar.py` : caméra gauche pour détection YOLO des couleurs
- `yolo_stereo.py` : paire stéréo pour estimation de profondeur par disparité

### Spécifications requises

| Paramètre | Minimum | Recommandé |
|---|---|---|
| Résolution par capteur | 640 × 480 | 1280 × 720 |
| Fréquence | 10 Hz | 30 Hz |
| Baseline (écartement) | 0,10 m | 0,40–0,80 m |
| Synchronisation | Software | **Hardware (trigger)** |
| Obturateur | Rolling | **Global shutter** |
| Champ de vision horizontal | 60° | 90° |
| Distance de travail fiable | 2–12 m | 1,5–15 m |
| Format | ROS2 `sensor_msgs/Image` | — |

!!! warning "Baseline et calibration"
    La baseline simulateur est `0,64 m`. Sur véhicule réel, adapter `BASELINE_M` dans `yolo_stereo.py` à la valeur mesurée. Une baseline plus courte réduit la précision à longue distance (Z ∝ B / disparité).

!!! info "Global shutter recommandé"
    Le rolling shutter déforme les objets en mouvement rapide. À 60 km/h, les cônes défilent vite — un global shutter évite les artefacts de détection.

### Paramètres de calibration à reconfigurer

```python
# Dans yolo_stereo.py
BASELINE_M   = 0.64    # ← mesurer sur le véhicule (en mètres)
FOCAL_PX     = 208.0   # ← lu automatiquement depuis /camera_info
```

Procédure de calibration stéréo recommandée : **ROS2 `camera_calibration`** avec damier 9×6.

### Modèles compatibles

| Modèle | Baseline | Global shutter | Driver ROS2 | Compatibilité |
|---|---|---|---|---|
| Intel RealSense D435i | 50 mm | Oui | `realsense2_camera` | ✅ Compact, intégré IMU |
| Intel RealSense D455 | 95 mm | Oui | `realsense2_camera` | ✅ Meilleure portée stéréo |
| ZED 2 (Stereolabs) | 120 mm | Non | `zed_ros2_wrapper` | ✅ SDK puissant, mais rolling shutter |
| ZED X (Stereolabs) | 120 mm | **Oui** | `zed_ros2_wrapper` | ✅ Recommandé si budget disponible |
| Rig personnalisé (2× e-CAM) | Configurable | Selon modèle | `usb_cam` × 2 | ⚠️ Sync hardware à prévoir |

---

## 3. Odométrie et localisation

### Rôle dans la stack

`odom_tf_publisher.py` souscrit à `/testing_only/odom` en simulation et publie le TF dynamique `fsds/map → fsds/FSCar`. Sur véhicule réel, ce topic doit être remplacé par une source d'odométrie fusionnée.

### Spécifications requises

| Paramètre | Valeur cible |
|---|---|
| Fréquence | ≥ 50 Hz |
| Précision position | < 0,5 m de dérive / tour de piste |
| Précision yaw (cap) | < 1° |
| Format | `nav_msgs/Odometry` (position + orientation quaternion) |

### Architecture recommandée

```
IMU (100–400 Hz) ──┐
                   ├──► robot_localization (EKF) ──► /odom  ──► odom_tf_publisher.py
GPS/RTK (1–10 Hz) ─┘         (fusionne avec GKIF/UKF)
```

| Composant | Spécification | Exemples |
|---|---|---|
| **IMU** | 100–400 Hz, drift < 0,1 °/s | VectorNav VN-100, Xsens MTi-3, ICM-42688-P |
| **GPS RTK** | Précision < 2 cm (RTK fix), 5–10 Hz | u-blox F9P, Piksi Multi, Septentrio mosaic-X5 |
| **Encodeurs roues** | Résolution > 500 ppr, 100 Hz | Encodeurs magnétiques AMT102 |
| **Nœud fusion** | `robot_localization` (EKF/UKF) | Package ROS2 standard |

!!! note "Sans RTK"
    Si le GPS RTK n'est pas disponible (ex. : piste couverte), utiliser **encodeurs + IMU** uniquement. La dérive reste acceptable sur un circuit court (< 200 m) si l'IMU est de bonne qualité.

### Topic à reconfigurer

```python
# Dans config.yaml
odometry: "/odom"          # remplace "/testing_only/odom"

# Dans odom_tf_publisher.py — frames à adapter
MAP_FRAME = "map"          # remplace "fsds/map"
CAR_FRAME = "base_link"    # remplace "fsds/FSCar"
```

---

## 4. Unité de calcul embarquée

### Exigences minimales

| Ressource | Minimum absolu | Recommandé |
|---|---|---|
| **GPU VRAM** | 2 Go | 4–8 Go |
| **RAM** | 8 Go | 16 Go |
| **CPU** | 4 cœurs @ 2 GHz | 6–8 cœurs @ 3 GHz+ |
| **Stockage** | 32 Go (SSD) | 64 Go NVMe |
| **OS** | Ubuntu 20.04 ou 22.04 | Ubuntu 22.04 |
| **Alimentation** | 30 W (NVIDIA Jetson) | 65–100 W (PC embarqué) |

!!! info "Mesure réelle avec `system_monitor.py`"
    Lancer `python3 system_monitor.py` pendant un test simulateur pour obtenir la consommation réelle RAM/CPU/VRAM de la stack. Utiliser les valeurs affichées comme base pour le dimensionnement matériel.

### Plateformes testées / recommandées

| Plateforme | GPU | VRAM | RAM | Adapté à |
|---|---|---|---|---|
| **NVIDIA Jetson Orin NX 16 Go** | 1024 CUDA cores (Ampere) | 16 Go unifiée | 16 Go | ✅ Recommandé embarqué |
| NVIDIA Jetson AGX Orin 32 Go | 2048 CUDA cores | 32 Go unifiée | 32 Go | ✅ Haut de gamme |
| NVIDIA Jetson Orin Nano 8 Go | 1024 CUDA cores | 8 Go unifiée | 8 Go | ⚠️ Juste suffisant (pipeline LiDAR seul) |
| PC embarqué + RTX 3060 Mobile | 3840 CUDA cores | 6 Go | 16–32 Go | ✅ Performances élevées, encombrant |
| Raspberry Pi 5 | VideoCore VII | Partagée | 8 Go | ❌ Pas de CUDA, YOLO trop lent |

### Estimation de la consommation (simulateur, pipeline LiDAR)

| Processus | RAM | CPU | VRAM |
|---|---|---|---|
| `yolo_lidar.py` | ~800 Mo | 1,5 cœur | ~400 Mo |
| `cone_mapper_lidar.py` | ~120 Mo | 0,3 cœur | — |
| `odom_tf_publisher.py` | ~80 Mo | 0,1 cœur | — |
| ROS2 middleware | ~200 Mo | 0,4 cœur | — |
| **Total pipeline LiDAR** | **~1,2 Go** | **~2,3 cœurs** | **~400 Mo** |
| **Total pipeline stéréo** | **~1,5 Go** | **~3,0 cœurs** | **~500 Mo** |

*Valeurs indicatives mesurées sur Ubuntu 22.04, RTX 3060, YOLOv8n.*

---

## 5. Montage et câblage recommandés

### Positions capteurs

```
           ← AVANT DU VÉHICULE →

    ┌──────────────────────────────────┐
    │   [CAM_G]  [LiDAR]  [CAM_D]     │  ← arceau ou proue (vue dégagée)
    │                                  │
    │         [IMU]  [GPS]             │  ← châssis rigide, loin des vibrations
    │                                  │
    │      [COMPUTE]  [BATTERIE]       │  ← habitacle
    └──────────────────────────────────┘
```

### Recommandations

- **LiDAR** : monter en hauteur (> 0,5 m du sol), axe de rotation vertical, dégagement 360° si possible
- **Caméras** : à la même hauteur, orientées vers l'avant, baseline maximale compatible avec le châssis
- **IMU** : vissé rigidement au châssis (éviter caoutchouc anti-vibration qui filtre aussi le signal utile)
- **GPS** : antenne en extérieur, dégagement ciel > 90°, câble court (< 3 m si possible)
- **Câblage** : Ethernet ou USB 3.0 pour LiDAR et caméras — éviter USB 2.0 pour le débit vidéo
