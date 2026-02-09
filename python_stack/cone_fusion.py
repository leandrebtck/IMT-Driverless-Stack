#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from visualization_msgs.msg import Marker, MarkerArray

# Couleur par défaut si YOLO pas disponible
DEFAULT_COLOR = "ORANGE"

class ConeFusion(Node):
    def __init__(self):
        super().__init__('cone_fusion')

        self.create_subscription(PointCloud2, '/lidar/obstacles', self.lidar_callback, 10)
        self.cones_pub = self.create_publisher(MarkerArray, '/cones_detected', 10)

        self.cluster_dist = 0.5
        self.min_points = 5

        self.get_logger().info("Cone Fusion node started")

    def lidar_callback(self, msg):
        points = np.array([
            [p[0], p[1], p[2]]
            for p in point_cloud2.read_points(msg, field_names=("x","y","z"), skip_nans=True)
            if -0.2 < p[2] < 0.5
        ])
        if len(points) == 0:
            return

        clusters = self.cluster_points(points)
        marker_array = MarkerArray()
        cone_id = 0

        for cluster in clusters:
            if len(cluster) < self.min_points:
                continue
            centroid = np.mean(cluster, axis=0)
            x, y = centroid[0], centroid[1]

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

            m.text = DEFAULT_COLOR  # ajout du label couleur
            marker_array.markers.append(m)
            cone_id += 1

        self.cones_pub.publish(marker_array)

    def cluster_points(self, points):
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
                if np.linalg.norm(points[i][:2]-points[j][:2]) < self.cluster_dist:
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
