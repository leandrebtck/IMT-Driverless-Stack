#!/usr/bin/env python3
"""
lidar_cluster.py — Clustering DBSCAN sur nuage de points LiDAR filtre.

Recoit les points au-dessus du sol depuis /lidar/obstacles (produit par
lidar_filtering.py) et regroupe les points en clusters correspondant
a des cones potentiels.

Publications :
  /lidar/cone_markers           — MarkerArray (visualisation RViz)
  /perception/lidar_detections  — Detection2DArray (entree cone_mapper_lidar.py)

Le LiDAR ne fournissant pas d'information couleur, class_id = -1 pour tous
les cones. Le cone_mapper_lidar.py les affichera en gris jusqu'a fusion
eventuelle avec YOLO.
"""

import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
from config_loader import CFG

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
from sklearn.cluster import DBSCAN
from visualization_msgs.msg import Marker, MarkerArray
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose

# ── Paramètres chargés depuis config.yaml ────────────────────────────────────
_db = CFG['dbscan']
DBSCAN_EPS           = _db['eps_m']
DBSCAN_MIN_SAMPLES   = _db['min_samples']
MAX_CONE_DIST_M      = _db['max_cone_distance_m']
MAX_CLUSTER_HEIGHT_M = _db['max_cluster_height_m']
MARKER_LIFETIME_NS   = 200_000_000
MARKER_SCALE         = 0.4


class LidarClusteringNode(Node):

    def __init__(self):
        super().__init__('lidar_clustering_node')

        _t = CFG['topics']
        self.subscription = self.create_subscription(
            PointCloud2,
            _t['lidar_filtered'],
            self.listener_callback,
            qos_profile_sensor_data
        )

        # MarkerArray pour la visualisation RViz
        self.pub_markers = self.create_publisher(
            MarkerArray, _t['lidar_cone_markers'], 10
        )

        # Detection2DArray pour cone_mapper_lidar.py
        # La position 3D est encodee dans hyp.pose.pose.position (repere LiDAR)
        self.pub_detections = self.create_publisher(
            Detection2DArray, _t['lidar_detections'], 10
        )

        # DBSCAN instancie une seule fois pour eviter la reallocation a chaque frame
        self.clustering_model = DBSCAN(eps=DBSCAN_EPS, min_samples=DBSCAN_MIN_SAMPLES)

        self.get_logger().info(
            f"LidarClusteringNode demarre — "
            f"eps={DBSCAN_EPS}m min_samples={DBSCAN_MIN_SAMPLES} "
            f"max_dist={MAX_CONE_DIST_M}m max_h={MAX_CLUSTER_HEIGHT_M}m"
        )

    def listener_callback(self, msg):
        # Conversion ROS PointCloud2 -> numpy (x, y, z)
        cloud_data = list(pc2.read_points(
            msg, field_names=("x", "y", "z"), skip_nans=True
        ))
        if not cloud_data:
            return

        points_3d = np.array(cloud_data, dtype=np.float32)
        if points_3d.shape[0] == 0:
            return

        # Filtrage par distance horizontale : rejette les points trop loin
        dist_xy = np.sqrt(points_3d[:, 0] ** 2 + points_3d[:, 1] ** 2)
        points_3d = points_3d[dist_xy < MAX_CONE_DIST_M]
        if points_3d.shape[0] == 0:
            return

        # Clustering en 2D (projection XY vue du dessus).
        # Travailler en 2D evite de separer un cone en deux clusters
        # si le LiDAR scanne differentes hauteurs du meme objet.
        points_2d  = points_3d[:, :2]
        clustering = self.clustering_model.fit(points_2d)
        labels     = clustering.labels_

        marker_array = MarkerArray()
        det_array    = Detection2DArray()
        det_array.header = msg.header

        for label in set(labels):
            if label == -1:
                # Bruit DBSCAN : points isoles qui n'appartiennent a aucun cluster
                continue

            mask           = (labels == label)
            cluster_points = points_3d[mask]

            # Validation geometrique : un cone ne doit pas depasser MAX_CLUSTER_HEIGHT_M
            cluster_height = cluster_points[:, 2].max() - cluster_points[:, 2].min()
            if cluster_height > MAX_CLUSTER_HEIGHT_M:
                continue

            # Centre de gravite du cluster = position estimee du cone
            center    = np.mean(cluster_points, axis=0)
            center_xy = float(np.sqrt(center[0] ** 2 + center[1] ** 2))

            # ── Marker RViz ──────────────────────────────────────────────
            marker                    = Marker()
            marker.header             = msg.header
            marker.ns                 = "lidar_clusters"
            marker.id                 = int(label)
            marker.type               = Marker.SPHERE
            marker.action             = Marker.ADD
            marker.pose.position.x    = float(center[0])
            marker.pose.position.y    = float(center[1])
            marker.pose.position.z    = float(center[2])
            marker.pose.orientation.w = 1.0
            marker.scale.x = marker.scale.y = marker.scale.z = MARKER_SCALE
            # Vert : detection LiDAR sans couleur (gris dans le mapper)
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 1.0
            marker.lifetime.nanosec = MARKER_LIFETIME_NS
            marker_array.markers.append(marker)

            # ── Detection2D pour cone_mapper_lidar ───────────────────────
            det        = Detection2D()
            det.header = msg.header

            hyp = ObjectHypothesisWithPose()

            # class_id = -1 : couleur inconnue (LiDAR ne voit pas les couleurs).
            # Le cone_mapper_lidar l'affichera en gris.
            hyp.hypothesis.class_id = "-1"

            # Score positif = detection instrumentale (LiDAR).
            # Encode la confiance en fonction de la distance :
            # plus le cone est proche, plus la mesure est fiable.
            hyp.hypothesis.score = float(
                max(0.1, 1.0 - center_xy / MAX_CONE_DIST_M)
            )

            # Position 3D dans le repere LiDAR (attendu par cone_mapper_lidar.py)
            hyp.pose.pose.position.x = float(center[0])
            hyp.pose.pose.position.y = float(center[1])
            hyp.pose.pose.position.z = float(center[2])

            det.results.append(hyp)
            det_array.detections.append(det)

        self.pub_markers.publish(marker_array)
        self.pub_detections.publish(det_array)

        if det_array.detections:
            self.get_logger().info(
                f"{len(det_array.detections)} cone(s) detecte(s) | "
                f"points bruts : {points_3d.shape[0]}",
                throttle_duration_sec=1.0
            )


def main(args=None):
    rclpy.init(args=args)
    node = LidarClusteringNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
