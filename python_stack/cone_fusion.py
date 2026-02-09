#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np

from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from visualization_msgs.msg import Marker, MarkerArray
from rclpy.qos import qos_profile_sensor_data


class ConeFusion(Node):
    def __init__(self):
        super().__init__('cone_fusion_node')

        # -------- SUBSCRIBER --------
        # On s'abonne au topic filtré du LiDAR
        self.subscription = self.create_subscription(
            PointCloud2,
            '/lidar/obstacles',
            self.lidar_callback,
            qos_profile_sensor_data
        )

        # -------- PUBLISHER --------
        self.cones_pub = self.create_publisher(
            MarkerArray,
            '/cones_relative',
            10
        )

        # -------- PARAMS CLUSTERING --------
        self.cluster_dist = 1.0   # distance max entre points d’un même cône
        self.min_points = 1       # points min pour valider un cône (pour debug)

        self.get_logger().info("🟢 Cone Fusion node started")

    # -----------------------------------------------------
    def lidar_callback(self, msg):
        # Convertir le PointCloud2 en numpy array
        points = np.array([
            [p[0], p[1], p[2]]
            for p in point_cloud2.read_points(msg, field_names=("x","y","z"), skip_nans=True)
        ])

        if points.shape[0] == 0:
            return

        # Debug : afficher nombre de points
        print(f"\nNombre de points LiDAR reçus: {len(points)}")
        print("Exemples de points (x,y,z) :", points[:10])

        # Clustering
        clusters = self.cluster_points(points)
        print(f"Nombre de clusters détectés: {len(clusters)}")

        marker_array = MarkerArray()
        cone_id = 0

        print("--- CONES DETECTED (vehicle frame) ---")
        for cluster in clusters:
            if len(cluster) < self.min_points:
                continue

            # Centroid du cluster = position du cône
            centroid = np.mean(cluster, axis=0)
            x, y = centroid[0], centroid[1]

            # Affichage dans le terminal pour le SLAM
            print(f"Cone {cone_id}: x = {x:.2f} m | y = {y:.2f} m | points in cluster: {len(cluster)}")

            # Marker RViz (optionnel pour visualisation)
            m = Marker()
            m.header.frame_id = "base_link"
            m.id = cone_id
            m.type = Marker.CYLINDER
            m.action = Marker.ADD
            m.scale.x = 0.3
            m.scale.y = 0.3
            m.scale.z = 0.5
            m.pose.position.x = x
            m.pose.position.y = y
            m.pose.position.z = 0.25
            m.color.a = 1.0
            m.color.r = 1.0
            m.color.g = 0.5
            m.color.b = 0.0

            marker_array.markers.append(m)
            cone_id += 1

        # Publication pour SLAM / RViz
        self.cones_pub.publish(marker_array)

    # -----------------------------------------------------
    def cluster_points(self, points):
        """
        Clustering simple basé sur distance euclidienne (x,y)
        """
        clusters = []
        used = np.zeros(len(points), dtype=bool)

        for i in range(len(points)):
            if used[i]:
                continue
            cluster = [points[i]]
            used[i] = True

            for j in range(i+1, len(points)):
                if used[j]:
                    continue
                if np.linalg.norm(points[i][:2] - points[j][:2]) < self.cluster_dist:
                    cluster.append(points[j])
                    used[j] = True

            clusters.append(cluster)

        return clusters


# ---------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = ConeFusion()
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
