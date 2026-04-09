# Repères TF (Coordinate Frames)

## Arbre TF complet

```
fsds/map  ← créé par odom_tf_publisher.py (n'existe PAS dans le bridge FSDS)
    └── fsds/FSCar  ← dynamique, mis à jour à chaque odométrie (~50 Hz)
            ├── fsds/cam1    ← TF statique, publié par le bridge
            ├── fsds/cam2    ← TF statique, publié par le bridge
            └── fsds/Lidar1  ← TF statique, publié par le bridge
```

!!! danger "Point critique"
    `fsds/map` **n'est pas publié par le bridge FSDS**. Il est créé uniquement par `odom_tf_publisher.py`.
    Sans ce nœud, toutes les transformations `tf_buffer.transform(..., 'fsds/map')` échouent
    et les SLAM ne peuvent pas construire de carte globale.

---

## Conventions d'axes par frame

| Frame | X | Y | Z | Notes |
|---|---|---|---|---|
| `fsds/map` | Est (avant au départ) | Gauche | Haut | Repère monde fixe |
| `fsds/FSCar` | Avant voiture | Gauche | Haut | Suit la pose voiture |
| `fsds/cam1` | Avant (profondeur) | Gauche | Haut | Convention physique FSDS |
| `fsds/cam2` | Avant (profondeur) | Gauche | Haut | Identique à cam1 |
| `fsds/Lidar1` | Avant | Gauche | Haut | Aligné avec la voiture |

!!! info "Convention optique vs physique"
    La convention standard optique (Z=profondeur, X=droite, Y=bas) est **différente** de la convention physique FSDS (X=avant, Y=gauche, Z=haut).
    Les transformations TF2 gèrent cette conversion automatiquement.

---

## Où est utilisé chaque repère

| Frame | Utilisé dans | Pour quoi |
|---|---|---|
| `fsds/map` | `cone_mapper*.py`, `odom_tf_publisher.py`, `cone_evaluator.py` | Carte globale persistante |
| `fsds/FSCar` | Bridge FSDS | Pose de la voiture |
| `fsds/cam1` | `yolo_stereo.py`, `yolo_lidar.py` | Détections caméra (position 3D relative) |
| `fsds/Lidar1` | `yolo_lidar.py`, `lidar_cluster.py`, `cone_mapper_lidar.py` | Points LiDAR, trackers Kalman |

---

## Transformation clé : LiDAR → Map

Utilisée dans `cone_mapper_lidar.py` à chaque nouvelle détection :

```python
pt = PointStamped()
pt.header.frame_id = 'fsds/Lidar1'
pt.header.stamp    = detection.header.stamp
pt.point.x = x_lidar
pt.point.y = y_lidar
pt.point.z = z_lidar

pt_map = tf_buffer.transform(pt, 'fsds/map')
# → position absolue dans le repère monde
```

Cette transformation combine :

1. `fsds/Lidar1 → fsds/FSCar` (TF statique, bridge)
2. `fsds/FSCar → fsds/map` (TF dynamique, odom_tf_publisher)
