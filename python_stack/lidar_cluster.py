#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
from sklearn.cluster import DBSCAN
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA

class LidarClusteringNode(Node):
    def __init__(self):
        super().__init__('lidar_clustering_node')

        # Topics
        self.input_topic = '/lidar/obstacles'     # Vient du noeud de filtrage sol
        self.marker_topic = '/lidar/cone_markers' # Sortie vers le mapping

        self.subscription = self.create_subscription(
            PointCloud2,
            self.input_topic,
            self.listener_callback,
            qos_profile_sensor_data
        )

        self.publisher = self.create_publisher(MarkerArray, self.marker_topic, 10)

        # DBSCAN : epsilon=0.5m (taille max d'un cone + marge), min_samples=3 points
        self.clustering_model = DBSCAN(eps=0.5, min_samples=3)

        self.get_logger().info("🧩 Clustering Node (DBSCAN) démarré.")

    def listener_callback(self, msg):
        # 1. Conversion ROS -> Numpy
        cloud_data = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
        points_3d = np.array(cloud_data)

        if points_3d.shape[0] == 0:
            return

        # 2. Clustering
        # On utilise seulement X et Y pour le clustering (projection 2D vue de dessus)
        # Cela évite de séparer un cône en deux si le lidar scanne le haut et le bas
        points_2d = points_3d[:, :2] 
        clustering = self.clustering_model.fit(points_2d)
        labels = clustering.labels_

        unique_labels = set(labels)
        marker_array = MarkerArray()

        # 3. Création des markers
        for label in unique_labels:
            if label == -1: continue # Bruit

            mask = (labels == label)
            cluster_points = points_3d[mask]

            # Centre de gravité du cluster (position du cône)
            center = np.mean(cluster_points, axis=0)

            marker = Marker()
            marker.header = msg.header # Garde le timestamp et frame_id (ex: velodyne)
            marker.ns = "lidar_clusters"
            marker.id = int(label)

            
            marker.type = Marker.SPHERE  # On veut une sphère pour différencier l'affichage par défault du simu et de nos calculs
            marker.action = Marker.ADD
            
            # Position
            marker.pose.position.x = center[0]
            marker.pose.position.y = center[1]
            marker.pose.position.z = center[2]
            
            marker.pose.orientation.w = 1.0
            
            # Taille
            marker.scale.x = 0.4
            marker.scale.y = 0.4
            marker.scale.z = 0.4

            # Couleur (VERT FLUO)
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 1.0 

            marker.lifetime.sec = 0
            marker.lifetime.nanosec = 200000000 # 0.2s de vie

            marker_array.markers.append(marker)

        self.publisher.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    node = LidarClusteringNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
