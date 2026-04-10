import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from config_loader import CFG

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
from std_msgs.msg import Header

class LidarFilteringNode(Node):
    def __init__(self):
        super().__init__('lidar_filtering_node')

        # Topics
        self.input_topic  = CFG['topics']['lidar']
        self.output_topic = CFG['topics']['lidar_filtered']
        self.Z_THRESHOLD  = CFG['lidar']['z_ground_threshold_m']

        # Subscriber 
        self.subscription = self.create_subscription(
            PointCloud2,
            self.input_topic,
            self.listener_callback,
            qos_profile_sensor_data
        )

        # Publisher 
        self.publisher = self.create_publisher(PointCloud2, self.output_topic, 10)

        self.get_logger().info(f"🧹 Filtre Sol Actif (Seuil Z > {self.Z_THRESHOLD}m)")

    def listener_callback(self, msg):
    
        # On convertit directement en numpy pour la vitesse de calcul
        cloud_data = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
        points = np.array(cloud_data)

        if points.shape[0] == 0:
            return

        # 2. FILTRAGE 
        # On garde uniquement les points dont la hauteur (Z) est supérieure au seuil
        # points[:, 2] correspond à la colonne Z
        mask = points[:, 2] > self.Z_THRESHOLD
        filtered_points = points[mask]

        # 3. Publication
        if len(filtered_points) > 0:
            # On crée un nouveau message PointCloud2
    
            header = Header()
            header.stamp = msg.header.stamp
            header.frame_id = msg.header.frame_id

            # Création du nuage filtré
            filtered_cloud = pc2.create_cloud_xyz32(header, filtered_points)
            
            self.publisher.publish(filtered_cloud)
            
            # Log pour vérifier (affiche 1 fois sur 10)
            self.get_logger().info(f"Points reçus: {len(points)} -> Gardés: {len(filtered_points)}")

def main(args=None):
    rclpy.init(args=args)
    node = LidarFilteringNode()
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