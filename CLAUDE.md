# IMT Driverless Stack — Contexte projet

## Vue d'ensemble
Stack de perception et SLAM pour Formula Student Driverless, développée sur le simulateur FSDS (Formula Student Driverless Simulator).
Compatible **ROS2 Iron** et **ROS2 Galactic**.

## Environnement simulateur
- **Simulateur** : FSDS (Formula Student Driverless Simulator)
  - Binaire : `~/Formula-Student-Driverless-Simulator-binary/FSDS.sh`
  - Bridge ROS2 : `~/Formula-Student-Driverless-Simulator/ros2`
- **LiDAR** : 3D multi-couches (Velodyne-style), publie `sensor_msgs/PointCloud2`
- **Caméras** : stéréo (cam1 gauche, cam2 droite), résolution typique 640×480
- **Capteurs disponibles** : LiDAR, caméra stéréo, IMU, GPS, odométrie, GSS

## Topics ROS2 principaux
| Topic | Type | Description |
|---|---|---|
| `/fsds/cam1/image_color` | `sensor_msgs/Image` | Caméra gauche |
| `/fsds/cam2/image_color` | `sensor_msgs/Image` | Caméra droite |
| `/fsds/cam1/camera_info` | `sensor_msgs/CameraInfo` | Infos intrinsèques cam1 |
| `/lidar/Lidar1` | `sensor_msgs/PointCloud2` | Nuage de points LiDAR 3D |
| `/testing_only/odom` | `nav_msgs/Odometry` | Odométrie ground truth |
| `/imu/Imu1` | `sensor_msgs/Imu` | IMU |
| `/perception/stereo_detections` | `vision_msgs/Detection2DArray` | Sorties yolo_stereo |
| `/perception/lidar_detections` | `vision_msgs/Detection2DArray` | Sorties yolo_lidar |
| `/slam/cone_map` | `visualization_msgs/MarkerArray` | Carte SLAM stéréo |
| `/slam_lidar/cone_map` | `visualization_msgs/MarkerArray` | Carte SLAM LiDAR |
| `/slam/car_path` | `nav_msgs/Path` | Trajectoire voiture |
| `/slam/stats` | `std_msgs/String` | Stats SLAM stéréo |
| `/slam_lidar/stats` | `std_msgs/String` | Stats SLAM LiDAR |

## Repères TF
| Frame | Description |
|---|---|
| `fsds/map` | Repère monde (publié par `odom_tf_publisher.py`) |
| `fsds/FSCar` | Repère voiture (dynamique, publié par `odom_tf_publisher.py`) |
| `fsds/cam1` | Caméra gauche (statique, bridge FSDS) |
| `fsds/cam2` | Caméra droite (statique, bridge FSDS) |
| `fsds/Lidar1` | LiDAR (statique, bridge FSDS) |

Convention repère physique fsds/cam1 : **X=avant, Y=gauche, Z=haut**
Convention repère optique caméra : Z=profondeur, X=droite, Y=bas

## Géométrie de piste Formula Student
- Largeur de piste : **6 m**
- Espacement entre cônes (même côté) : **~5 m** (moins en virage)
- Nombre de tours max : **10**
- Cônes jaunes : côté droit
- Cônes bleus : côté gauche
- Cônes orange : départ/arrivée (des deux côtés)
- Classes YOLO : `0=JAUNE, 1=BLEU, 2=ORANGE`

## Fichiers Python principaux (`python_stack/`)
| Fichier | Description |
|---|---|
| `yolo_stereo.py` | Détection YOLO stéréo + estimation profondeur (stéréo + mono fallback) |
| `yolo_lidar.py` | Détection YOLO + LiDAR avec filtre de Kalman par cône |
| `cone_mapper.py` | SLAM stéréo : carte globale depuis `/perception/stereo_detections` |
| `cone_mapper_lidar.py` | SLAM LiDAR : carte globale depuis `/perception/lidar_detections` |
| `odom_tf_publisher.py` | Publie TF `fsds/map → fsds/FSCar` depuis `/testing_only/odom` |
| `global_drive.py` | Contrôle voiture au clavier (Z=avant, S=arrière, Q=gauche, D=droite) |
| `sensor_fusion.py` | Fusion capteurs (référence de l'architecture) |

## Scripts de lancement (`python_stack/`)
| Script | Description |
|---|---|
| `slam_stereo.sh` | FSDS + Bridge + YOLO stéréo + SLAM stéréo + RViz + Drive |
| `slam_lidar.sh` | FSDS + Bridge + YOLO LiDAR + SLAM LiDAR + RViz + Drive |
| `auto_launch.sh` | Lancement basique YOLO caméra seule |
| `auto_stereo.sh` | Lancement YOLO stéréo sans SLAM |
| `auto_yolo_lidar.sh` | Lancement YOLO + LiDAR sans SLAM |

## Launcher GUI
```bash
python3 ~/IMT-Driverless-Stack/launcher.py
```
Liste tous les `.sh` du repo, double-clic pour lancer.

## Workspace ROS2
```bash
cd ~/IMT-Driverless-Stack/ros_workspace
colcon build
source install/setup.bash
```

## Paramètres clés SLAM LiDAR (`cone_mapper_lidar.py`)
- `MERGE_DIST = 1.5 m` — distance max pour fusionner deux détections
- `COUNT_CAP = 20` — fenêtre de la moyenne pondérée (stabilise sur 10 tours)
- `MAX_DEPTH_STRAIGHT = 8.0 m` — profondeur max (LiDAR fiable)
- `MAX_DEPTH_TURN = 5.0 m` — profondeur max en virage
- `TURN_YAW_RATE = 0.3 rad/s` — seuil détection virage (depuis `/testing_only/odom`)
- `LINE_STRIDE = 1` — tous les cônes comme points de contrôle (LiDAR précis)
- `MAX_LINE_STEP = 8.0 m` — gap max entre deux cônes dans une ligne

## Paramètres clés YOLO LiDAR (`yolo_lidar.py`)
- `MIN_HITS = 3` — détections min pour confirmer un cône (anti faux positifs)
- `MAX_MISSES = 8` — frames sans détection avant suppression du tracker
- `ASSOC_DIST = 1.5 m` — distance 3D max pour associer à un tracker existant
- `OUTLIER_STD = 0.5 m` — rejet outliers LiDAR (± médiane de distance)
- Kalman : `R_std=0.20 m` (bruit mesure LiDAR), `Q_std=0.02 m` (bruit processus)

## Paramètres clés SLAM stéréo (`cone_mapper.py`)
- `MAX_DEPTH_STRAIGHT = 4.0 m` (stéréo moins fiable que LiDAR à longue distance)
- `MAX_DEPTH_TURN = 4.0 m`
- `LINE_STRIDE = 3` — 1 cône sur 3 (moins précis → lissage plus fort)
- Tri des lignes : **par ordre de détection (ID croissant)** = ordre de conduite

## Notes importantes
- `fsds/map` n'existe pas nativement dans le TF du bridge FSDS → créé par `odom_tf_publisher.py`
- Le bridge FSDS publie uniquement les TF statiques des capteurs, pas la pose dynamique de la voiture
- Compatibilité Galactic/Iron : `try: import sensor_msgs_py.point_cloud2 except: from sensor_msgs import point_cloud2`
- En virage (`|yaw_rate| ≥ 0.3 rad/s`) : aucun nouveau cône ajouté à la carte (positions peu fiables)
- Lignes orange non tracées dans SLAM LiDAR (trop peu de cônes → instable)
- Repo GitHub : https://github.com/leandrebtck/IMT-Driverless-Stack
