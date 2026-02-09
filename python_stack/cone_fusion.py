#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
import math
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import Odometry

class ConeFusion(Node):
    def __init__(self):
        super().__init__('cone_fusion')

        # Abonnement aux clusters (local) et à l'odométrie
        self.create_subscription(MarkerArray, '/lidar/cone_markers', self.cluster_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # Publication de la CARTE GLOBALE (persistante)
        self.global_map_pub = self.create_publisher(MarkerArray, '/map/global_cones', 10)

        # État du véhicule
        self.car_x = 0.0
        self.car_y = 0.0
        self.car_yaw = 0.0

        # Base de données des cônes globaux
        # Liste de dictionnaires : [{'x': 12.5, 'y': 8.2, 'color': 'unknown'}, ...]
        self.global_cones = []
        self.ASSOCIATION_DIST = 1.0 # Rayon (m) pour dire "c'est le même cône"

        self.get_logger().info("🌍 Cone Fusion (Mapping) Node started")

    def odom_callback(self, msg: Odometry):
        self.car_x = msg.pose.pose.position.x
        self.car_y = msg.pose.pose.position.y
        
        # Quaternion -> Euler (Yaw)
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.car_yaw = math.atan2(siny_cosp, cosy_cosp)

    def cluster_callback(self, msg: MarkerArray):
        # On attend d'avoir une position valide avant de mapper
        if self.car_x == 0.0 and self.car_y == 0.0:
            return

        new_detection = False

        for marker in msg.markers:
            # 1. Coordonnées locales (vu par le lidar)
            x_local = marker.pose.position.x
            y_local = marker.pose.position.y

            # 2. Transformation Locale -> Globale
            # X_global = X_car + (x_loc * cos(theta) - y_loc * sin(theta))
            x_global = self.car_x + (x_local * math.cos(self.car_yaw) - y_local * math.sin(self.car_yaw))
            y_global = self.car_y + (x_local * math.sin(self.car_yaw) + y_local * math.cos(self.car_yaw))

            # 3. Association de données (Est-ce un nouveau cône ?)
            if self.is_new_cone(x_global, y_global):
                self.global_cones.append({'x': x_global, 'y': y_global, 'color': 'unknown'})
                new_detection = True

        # 4. Si on a trouvé de nouveaux cônes, on republie toute la carte
        if new_detection:
            self.publish_global_map()

    def is_new_cone(self, x, y):
        """Vérifie si un cône existe déjà à proximité de (x, y)"""
        for cone in self.global_cones:
            dist = math.sqrt((cone['x'] - x)**2 + (cone['y'] - y)**2)
            if dist < self.ASSOCIATION_DIST:
                return False # Cône déjà connu
        return True

    def publish_global_map(self):
        marker_array = MarkerArray()
        
        for i, cone in enumerate(self.global_cones):
            m = Marker()
            m.header.frame_id = "odom" # La carte est fixe dans le repère Odom
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = "global_map"
            m.id = i
            m.type = Marker.CYLINDER
            m.action = Marker.ADD
            m.pose.position.x = cone['x']
            m.pose.position.y = cone['y']
            m.pose.position.z = 0.2
            m.scale.x, m.scale.y, m.scale.z = 0.2, 0.2, 0.5
            
            # Couleur Orange (en attendant la fusion caméra)
            m.color.r = 1.0
            m.color.g = 0.5
            m.color.b = 0.0
            m.color.a = 1.0
            
            m.lifetime.sec = 0 # Infini
            marker_array.markers.append(m)

        self.global_map_pub.publish(marker_array)

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
