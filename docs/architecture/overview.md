# Architecture — Vue d'ensemble

La stack est organisée en **deux pipelines parallèles** (stéréo et LiDAR) partageant la même infrastructure TF et pouvant fonctionner simultanément ou indépendamment.

## Schéma général

```
┌─────────────────────────────────────────────────────────────────┐
│                        SIMULATEUR FSDS                          │
│   Caméra stéréo (cam1/cam2) │ LiDAR 3D │ Odométrie │ IMU      │
└──────────┬──────────────────────┬───────────────────┬───────────┘
           │   ROS2 Bridge        │                   │
           ▼                     ▼                   ▼
┌─────────────────┐   ┌──────────────────┐   ┌────────────────────┐
│  yolo_stereo.py │   │  yolo_lidar.py   │   │ odom_tf_publisher  │
│ (YOLO + stéréo) │   │ (YOLO + Kalman)  │   │  (TF fsds/map)     │
└────────┬────────┘   └────────┬─────────┘   └────────────────────┘
         │                     │
/perception/           /perception/
stereo_detections      lidar_detections
         │                     │
         ▼                     ▼
┌─────────────────┐   ┌──────────────────────┐
│  cone_mapper.py │   │ cone_mapper_lidar.py  │
│  (SLAM stéréo)  │   │    (SLAM LiDAR)       │
└────────┬────────┘   └──────────┬────────────┘
         │                       │
/slam/cone_map          /slam_lidar/cone_map
/slam/stats             /slam_lidar/stats
         │                       │
         └──────────┬────────────┘
                    ▼
         ┌─────────────────────┐      ┌───────────────────┐
         │  cone_evaluator.py  │ ←──  │ /testing_only/    │
         │  (comparaison GT)   │      │      track        │
         └────────┬────────────┘      └───────────────────┘
                  │
         /evaluation/stats
         /evaluation/markers
                  │
                  ▼
         ┌─────────────────┐
         │ eval_dashboard  │
         │  (GUI tkinter)  │
         └─────────────────┘
```

## Composants

| Composant | Fichier | Rôle |
|---|---|---|
| Détection stéréo | `yolo_stereo.py` | YOLO + profondeur stéréo/mono |
| Détection LiDAR | `yolo_lidar.py` | YOLO + fusion LiDAR + Kalman |
| Clustering LiDAR | `lidar_cluster.py` | DBSCAN sans YOLO |
| SLAM stéréo | `cone_mapper.py` | Carte globale depuis stéréo |
| SLAM LiDAR | `cone_mapper_lidar.py` | Carte globale depuis LiDAR *(recommandé)* |
| TF publisher | `odom_tf_publisher.py` | Crée le repère `fsds/map` |
| Évaluateur | `cone_evaluator.py` | Comparaison avec ground truth |
| Dashboard éval | `eval_dashboard.py` | GUI précision SLAM |
| Dashboard ressources | `system_monitor.py` | GUI RAM/CPU/VRAM |
| Contrôle clavier | `global_drive.py` | Conduite manuelle |
| Launcher | `launcher.py` | Interface de lancement |
