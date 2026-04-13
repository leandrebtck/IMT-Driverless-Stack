# Architecture — Vue d'ensemble

La stack est organisée en **trois dossiers fonctionnels** (`python_scripts/1perception/`, `python_scripts/2slam/`, `python_scripts/3control/`) partageant la même infrastructure TF et pouvant fonctionner simultanément ou indépendamment.

## Structure des dossiers

```
IMT-Driverless-Stack/
├── python_scripts/
│   ├── 1perception/     ← détection YOLO, fusion LiDAR, capteurs
│   ├── 2slam/           ← cartographie globale des cônes
│   ├── 3control/        ← pilotage autonome, conduite manuelle
│   └── 4performance/    ← dashboards évaluation et monitoring
├── scripts/             ← scripts de lancement (.sh)
├── config/              ← config.yaml, configs RViz
├── docs/                ← documentation
├── ros_workspace/       ← messages ROS2 (fs_msgs)
├── config_loader.py     ← lecture centralisée de config.yaml
└── launcher.py          ← interface graphique de lancement
```

## Schéma général

```
┌─────────────────────────────────────────────────────────────────┐
│                        SIMULATEUR FSDS                          │
│   Caméra stéréo (cam1/cam2) │ LiDAR 3D │ Odométrie │ IMU      │
└──────────┬──────────────────────┬───────────────────┬───────────┘
           │   ROS2 Bridge        │                   │
           ▼                     ▼                   ▼
┌──────────────────────────────────────────────────────────────────┐
│                  python_scripts/1perception/                     │
│  yolo_stereo.py │  yolo_lidar.py  │  odom_tf_publisher.py       │
│  lidar_ros.py   │  lidar_cluster.py │  sensor_fusion.py         │
└────────┬─────────────────┬────────────────────────────────────────┘
         │                 │
/perception/       /perception/
stereo_detections  lidar_detections
         │                 │
         ▼                 ▼
┌──────────────────────────────────────────────────────────────────┐
│                    python_scripts/2slam/                         │
│  cone_mapper.py          │  cone_mapper_lidar.py                │
│  cone_mapper_skidpad_stereo.py │ cone_mapper_skidpad_lidar.py   │
└────────┬─────────────────┬──────────────────────────────────────┘
         │                 │
/slam/cone_map    /slam_lidar/cone_map
/slam_skidpad_stereo/cone_map
/slam_skidpad_lidar/cone_map
         │                 │
         └────────┬─────────┘
                  ▼
┌──────────────────────────────────────────────────────────────────┐
│                         control/                                 │
│  centerline_follower.py  │  skidpad_driver.py                   │
│  acceleration_driver.py  │  global_drive.py                     │
└──────────────────────────────────────────────────────────────────┘
```

## Composants

### python_scripts/1perception/

| Fichier | Rôle |
|---|---|
| `yolo_stereo.py` | YOLO + profondeur stéréo/mono |
| `yolo_lidar.py` | YOLO + fusion LiDAR + Kalman |
| `yolo_ros.py` | Nœud YOLO caméra seule |
| `lidar_ros.py` | Filtrage sol PointCloud2 |
| `lidar_cluster.py` | Clustering DBSCAN sans YOLO |
| `odom_tf_publisher.py` | Crée le repère `fsds/map` |
| `sensor_fusion.py` | Référence architecture multi-capteurs |
| `cone_evaluator.py` | Comparaison SLAM vs ground truth |

### python_scripts/2slam/

| Fichier | Rôle |
|---|---|
| `cone_mapper_lidar.py` | SLAM LiDAR — carte globale *(recommandé)* |
| `cone_mapper.py` | SLAM stéréo — carte globale |
| `cone_mapper_skidpad_lidar.py` | SLAM LiDAR dédié Skidpad (centerline jaune↔bleu) |
| `cone_mapper_skidpad_stereo.py` | SLAM stéréo dédié Skidpad |

### python_scripts/3control/

| Fichier | Rôle |
|---|---|
| `centerline_follower.py` | Pilotage autonome circuit — suit la centerline SLAM |
| `skidpad_driver.py` | Pilotage autonome Skidpad — géométrie pure + Pure Pursuit |
| `acceleration_driver.py` | Pilotage autonome accélération — 75 m plein gaz |
| `global_drive.py` | Conduite manuelle clavier (Z/Q/S/D) |

### python_scripts/performance/

| Fichier | Rôle |
|---|---|
| `eval_dashboard.py` | Dashboard précision SLAM (jauge graphique tkinter) |
| `system_monitor.py` | Dashboard RAM/CPU/VRAM temps réel |
