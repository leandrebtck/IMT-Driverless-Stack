# Déploiement sur véhicule réel

## Correspondance topics FSDS → capteurs réels

Pour passer du simulateur au véhicule physique, remplacer les topics FSDS par les équivalents réels :

| Topic FSDS | Topic réel | Driver ROS2 |
|---|---|---|
| `/fsds/cam1/image_color` | `/camera/left/image_raw` | `usb_cam`, `realsense2_camera` |
| `/fsds/cam2/image_color` | `/camera/right/image_raw` | `usb_cam`, `realsense2_camera` |
| `/lidar/Lidar1` | `/velodyne_points` | `velodyne_driver` |
| `/lidar/Lidar1` | `/ouster/points` | `ouster_ros` |
| `/testing_only/odom` | `/odom` | `robot_localization`, `nmea_navsat` |
| *(TF statiques capteurs)* | Fichier URDF | `robot_state_publisher` |

!!! warning "Ground truth uniquement en simulation"
    Le topic `/testing_only/track` (vérité terrain des cônes) n'existe **pas** sur le véhicule réel.
    Le nœud `cone_evaluator.py` ne fonctionnera donc pas en conditions réelles.

---

## Nœuds compatibles sans modification

Ces nœuds fonctionnent sur véhicule réel en adaptant uniquement les noms de topics :

| Nœud | Adaptation nécessaire |
|---|---|
| `yolo_lidar.py` | Changer les topics caméra et LiDAR |
| `yolo_stereo.py` | Changer les topics caméra gauche/droite, recalibrer `BASELINE_M` et `FOCAL_PX` |
| `cone_mapper_lidar.py` | Changer le nom du frame LiDAR |
| `lidar_cluster.py` | Changer le topic d'entrée LiDAR |
| `odom_tf_publisher.py` | Changer le topic d'odométrie |
| `global_drive.py` | Remplacer `fs_msgs/ControlCommand` par le protocole du véhicule réel |

---

## Estimation des ressources embarquées

Utiliser `system_monitor.py` pendant les tests simulateur pour mesurer la consommation réelle de la stack.

Le dashboard affiche 3 catégories :

| Catégorie | Ce qu'il faut mesurer |
|---|---|
| Scripts Python perception | RAM + CPU des nœuds YOLO, SLAM, Kalman |
| Middleware ROS2 | Overhead de communication inter-processus |
| Drivers capteurs | RAM + CPU des drivers Velodyne/RealSense/etc. |

### Critères de choix ordinateur embarqué

| Ressource | Minimum recommandé |
|---|---|
| RAM | Total mesuré + 2 Go de marge OS |
| CPU | Total mesuré en cœurs × 1.3 (marge charge pics) |
| GPU VRAM | YOLO model (~300 Mo) + inférence (~200 Mo) = ~512 Mo minimum |
| Stockage | 20 Go (OS + ROS2 + poids YOLO + logs) |

---

## Calibration caméra stéréo (véhicule réel)

Paramètres à recalibrer dans `yolo_stereo.py` :

```python
BASELINE_M  = 0.64   # ← distance réelle entre les deux caméras (mesurer)
FOCAL_PX    = 208.0  # ← mis à jour automatiquement depuis /camera_info
DEPTH_OFFSET = {0: 0.0, 1: 0.0, 2: 0.0}  # ← correction empirique à recalibrer
```

Utiliser `ros2 topic echo /fsds/cam1/camera_info` (ou équivalent réel) pour récupérer la focale réelle.

---

## Workspace ROS2 (`ros_workspace/`)

Le package `fs_msgs` est compilé localement et spécifique au simulateur FSDS.
Sur véhicule réel, le remplacer par les messages standard ROS2 :

| `fs_msgs` | Équivalent standard |
|---|---|
| `fs_msgs/ControlCommand` | `geometry_msgs/Twist` ou protocole CAN |
| `fs_msgs/Track` | Pas d'équivalent (simulation uniquement) |
| `fs_msgs/Cone` | Pas d'équivalent (simulation uniquement) |
