#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import Odometry

# Paramètres
CLUSTER_DIST = 0.5  # Distance pour regrouper points LiDAR
MIN_POINTS = 5      # Minimum de points pour valider un cluster
DEFAULT_COLOR = "ORANGE"

class ConeFusion(Node):
    def __init__(self):
        super().__init__('cone_fusion')

        # Subscribers LiDAR et Odom
        self.create_subscription(PointCloud2, '/lidar/obstacles', self.lidar_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # Publisher pour les cônes fusionnés
        self.cones_pub = self.create_publisher(MarkerArray, '/cones_relative', 10)

        # Stockage
        self.cones_global = set()   # coordonnées globales (x, y)
        self.car_pos = (0.0, 0.0)
        self.car_yaw = 0.0          # orientation de la voiture

        self.get_logger().info("Cone Fusion node started")

    def odom_callback(self, msg: Odometry):
        self.car_pos = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        # Calcul de l'orientation yaw (z)
        import math
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.car_yaw = math.atan2(siny_cosp, cosy_cosp)

    def lidar_callback(self, msg: PointCloud2):
        # Lecture points LiDAR filtrés par Z
        points = np.array([
            [p[0], p[1]]
            for p in point_cloud2.read_points(msg, field_names=("x","y","z"), skip_nans=True)
            if -0.2 < p[2] < 0.5
        ])
        if len(points) == 0:
            return

        clusters = self.cluster_points(points)

        marker_array = MarkerArray()
        cone_id = 0

        for cluster in clusters:
            if len(cluster) < MIN_POINTS:
                continue
            centroid_local = np.mean(cluster, axis=0)
            # Transformation locale -> global
            x_global, y_global = self.local_to_global(centroid_local[0], centroid_local[1])

            key = (round(x_global, 2), round(y_global, 2))
            if key in self.cones_global:
                continue  # Ne pas republier si déjà détecté

            self.cones_global.add(key)

            # Création du Marker
            m = Marker()
            m.header.frame_id = "odom"
            m.id = cone_id
            m.type = Marker.CYLINDER
            m.action = Marker.ADD
            m.scale.x = 0.3
            m.scale.y = 0.3
            m.scale.z = 0.5
            m.pose.position.x = x_global
            m.pose.position.y = y_global
            m.pose.position.z = 0.25
            m.color.a = 1.0
            m.color.r = 1.0
            m.color.g = 0.0
            m.color.b = 0.0
            m.text = DEFAULT_COLOR

            marker_array.markers.append(m)
            cone_id += 1

        if marker_array.markers:
            self.cones_pub.publish(marker_array)

    def local_to_global(self, x_local, y_local):
        """Transforme les coordonnées LiDAR locales en coordonnées globales via Odom/GPS"""
        import math
        cos_yaw = math.cos(self.car_yaw)
        sin_yaw = math.sin(self.car_yaw)
        x_global = self.car_pos[0] + x_local * cos_yaw - y_local * sin_yaw
        y_global = self.car_pos[1] + x_local * sin_yaw + y_local * cos_yaw
        return x_global, y_global

    def cluster_points(self, points):
        """Regroupe points proches (simple clustering)"""
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
                if np.linalg.norm(points[i]-points[j]) < CLUSTER_DIST:
                    cluster.append(points[j])
                    used[j] = True
            clusters.append(cluster)
        return clusters

def main():
    rclpy.init()
    node = ConeFusion()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
