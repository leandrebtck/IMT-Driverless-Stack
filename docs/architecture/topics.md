# Topics ROS2

## Topics publiés par le Bridge FSDS

| Topic | Type | Description | Frame |
|---|---|---|---|
| `/fsds/cam1/image_color` | `sensor_msgs/Image` | Caméra gauche (640×480, RGB8) | `fsds/cam1` |
| `/fsds/cam2/image_color` | `sensor_msgs/Image` | Caméra droite (640×480, RGB8) | `fsds/cam2` |
| `/fsds/cam1/camera_info` | `sensor_msgs/CameraInfo` | Paramètres intrinsèques cam1 | `fsds/cam1` |
| `/fsds/cam2/camera_info` | `sensor_msgs/CameraInfo` | Paramètres intrinsèques cam2 | `fsds/cam2` |
| `/lidar/Lidar1` | `sensor_msgs/PointCloud2` | Nuage de points LiDAR 3D | `fsds/Lidar1` |
| `/testing_only/odom` | `nav_msgs/Odometry` | Odométrie ground truth | `fsds/map` |
| `/testing_only/track` | `fs_msgs/Track` | Positions réelles de tous les cônes | `fsds/map` |
| `/imu/Imu1` | `sensor_msgs/Imu` | Accéléromètre + gyroscope | `fsds/FSCar` |

---

## Topics publiés par `odom_tf_publisher.py`

| Topic | Type | Description |
|---|---|---|
| `/slam/car_path` | `nav_msgs/Path` | Trajectoire complète de la voiture |
| *(TF)* | `tf2_msgs/TFMessage` | Transform `fsds/map → fsds/FSCar` (~50 Hz) |

---

## Topics publiés par `yolo_stereo.py`

| Topic | Type | Description |
|---|---|---|
| `/perception/stereo_detections` | `vision_msgs/Detection2DArray` | Cônes détectés avec position 3D |
| `/yolo_stereo/debug_image` | `sensor_msgs/Image` | Image annotée avec bbox + profondeur |

### Format `Detection2DArray`

```python
header.frame_id = 'fsds/cam1'
results[i]:
  hypothesis.class_id = "0"/"1"/"2"   # JAUNE / BLEU / ORANGE
  hypothesis.score    = confidence YOLO
  pose.pose.position.x = X dans fsds/cam1
  pose.pose.position.y = Y dans fsds/cam1
  pose.pose.position.z = profondeur estimée (m)
```

---

## Topics publiés par `yolo_lidar.py`

| Topic | Type | Description |
|---|---|---|
| `/perception/lidar_detections` | `vision_msgs/Detection2DArray` | Cônes confirmés (position Kalman) |
| `/yolo_lidar/debug_image` | `sensor_msgs/Image` | Image avec bbox + points LiDAR projetés |
| `/yolo_lidar/cone_markers` | `visualization_msgs/MarkerArray` | Marqueurs 3D dans RViz |

---

## Topics publiés par `lidar_cluster.py`

| Topic | Type | Description |
|---|---|---|
| `/lidar/cone_markers` | `visualization_msgs/MarkerArray` | Clusters bruts (sphères vertes RViz) |
| `/perception/lidar_detections` | `vision_msgs/Detection2DArray` | Détections sans couleur (`class_id=-1`) |

---

## Topics publiés par `cone_mapper_lidar.py`

| Topic | Type | QoS | Description |
|---|---|---|---|
| `/slam_lidar/cone_map` | `visualization_msgs/MarkerArray` | **Latché** (TRANSIENT_LOCAL) | Carte globale LiDAR dans `fsds/map` |
| `/slam_lidar/stats` | `std_msgs/String` | Best effort | Statistiques SLAM texte |

### Namespaces dans `/slam_lidar/cone_map`

| `ns` | `type` | Description |
|---|---|---|
| `cone_map` | `CYLINDER` | Cônes de la carte |
| `cone_labels` | `TEXT_VIEW_FACING` | Compteur de détections sur chaque cône |
| `yellow_line` | `LINE_STRIP` | Ligne Catmull-Rom côté droit |
| `blue_line` | `LINE_STRIP` | Ligne Catmull-Rom côté gauche |

---

## Topics publiés par `cone_mapper.py` (SLAM stéréo)

| Topic | Type | QoS | Description |
|---|---|---|---|
| `/slam/cone_map` | `visualization_msgs/MarkerArray` | Latché | Carte globale stéréo dans `fsds/map` |
| `/slam/stats` | `std_msgs/String` | Best effort | Statistiques SLAM stéréo |

---

## Topics publiés par `cone_evaluator.py`

| Topic | Type | Description |
|---|---|---|
| `/evaluation/stats` | `std_msgs/String` | Métriques parsées par `eval_dashboard.py` |
| `/evaluation/markers` | `visualization_msgs/MarkerArray` | Flèches d'erreur + labels dans RViz |

### Format `/evaluation/stats`

```
"Comparaison 42/45 cônes | moy=0.123m | RMSE=0.156m | min=0.012m max=0.487m | fantômes=3"
```

---

## Topics publiés par `global_drive.py`

| Topic | Type | Description |
|---|---|---|
| `/control_command` | `fs_msgs/ControlCommand` | Commandes moteur/direction (10 Hz) |

---

## Messages personnalisés `fs_msgs`

Compilés dans `ros_workspace/src/fs_msgs/` :

| Message | Champs | Utilisé par |
|---|---|---|
| `fs_msgs/Track` | `track: Cone[]` | `/testing_only/track` |
| `fs_msgs/Cone` | `location: Point`, `color: uint8` | `cone_evaluator.py` |
| `fs_msgs/ControlCommand` | `throttle`, `brake`, `steering: float32` | `global_drive.py` |

### Constantes de couleur `fs_msgs/Cone`

!!! warning "Convention inversée par rapport à YOLO"
    FSDS et le modèle YOLO n'utilisent pas le même encodage des couleurs.

| `fs_msgs` | Valeur | Classe YOLO | Valeur YOLO |
|---|---|---|---|
| `BLUE` | 0 | BLEU | 1 |
| `YELLOW` | 1 | JAUNE | 0 |
| `ORANGE_BIG` | 2 | ORANGE | 2 |
| `ORANGE_SMALL` | 3 | ORANGE | 2 |
| `UNKNOWN` | 4 | — | -1 |
