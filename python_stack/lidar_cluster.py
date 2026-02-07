import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
from sklearn.cluster import DBSCAN
from visualization_msgs.msg import Marker, MarkerArray


class LidarClusteringNode(Node):
    def __init__(self):
        super().__init__('lidar_clustering_node')

        # Topics
        self.input_topic = '/lidar/obstacles'     
        self.marker_topic = '/lidar/cone_markers' 

        self.subscription = self.create_subscription(
            PointCloud2,
            self.input_topic,
            self.listener_callback,
            qos_profile_sensor_data
        )

        self.publisher = self.create_publisher(MarkerArray, self.marker_topic, 10)

        # PARAMETRES DBSCAN
        # eps : Distance max (0.6m) pour lier des points
        # min_samples : Au moins 3 points pour valider un objet
        self.clustering_model = DBSCAN(eps=0.6, min_samples=3)

        self.get_logger().info("🧩 Clustering Node démarré (DBSCAN)")

    def listener_callback(self, msg):
        # 1. Conversion ROS -> Numpy
        
        cloud_data = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
        points_3d = np.array(cloud_data)

        # Sécurité : Si le nuage est vide, on arrête 
        if points_3d.shape[0] == 0:
            return

        # 2. CLUSTERING
        # On fit le modèle sur les données
        clustering = self.clustering_model.fit(points_3d)
        labels = clustering.labels_

        # labels est une liste type [-1, 0, 0, 1, 1, 2...]
        unique_labels = set(labels)
        
        marker_array = MarkerArray()

        # 3. TRAITEMENT PAR GROUPE
        for label in unique_labels:
            if label == -1:
                continue # -1 = Bruit (points isolés), on ignore

            # Masque pour extraire uniquement les points du groupe actuel
            mask = (labels == label)
            cluster_points = points_3d[mask]

            # Calcul du centre de gravité 
            center_x = np.mean(cluster_points[:, 0])
            center_y = np.mean(cluster_points[:, 1])
            center_z = np.mean(cluster_points[:, 2])

            # 4. CRÉATION DU MARQUEUR
            marker = Marker()
            marker.header = msg.header 
            marker.ns = "cones"
            
        
            marker.id = int(label) # On force la conversion numpy -> int python
            
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            
            # Position
            marker.pose.position.x = center_x
            marker.pose.position.y = center_y
            marker.pose.position.z = center_z
            
            # Orientation (Neutre)
            marker.pose.orientation.w = 1.0

            # Taille (Sphère de 30cm)
            marker.scale.x = 0.3
            marker.scale.y = 0.3
            marker.scale.z = 0.3

            # Couleur (Vert Fluo)
            marker.color.a = 1.0 # Alpha (Visible)
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0

            # Durée de vie (0.2s) pour éviter les fantômes si le cône disparaît
            marker.lifetime.sec = 0
            marker.lifetime.nanosec = 200000000 

            marker_array.markers.append(marker)

        # 5. Publication
        self.publisher.publish(marker_array)
        # self.get_logger().info(f"✅ {len(marker_array.markers)} objets détectés.")

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