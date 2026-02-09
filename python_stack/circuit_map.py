#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import Odometry
from math import atan2, cos, sin

class GlobalCircuit(Node):
    def __init__(self):
        super().__init__('global_circuit_node')

        # Subscriber
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(MarkerArray, '/cones_relative', self.cones_callback, 10)

        # Publishers
        self.global_cones_pub = self.create_publisher(MarkerArray, '/cones_global', 10)
        self.vehicle_pub = self.create_publisher(MarkerArray, '/vehicle_marker', 10)

        # Stockage de la position et orientation du véhicule
        self.x_vehicle = 0.0
        self.y_vehicle = 0.0
        self.theta = 0.0

        # Liste globale des cônes uniques pour éviter les doublons
        self.global_cones = []

        self.get_logger().info("Global Circuit node started")

    def odom_callback(self, msg: Odometry):
        # Position du véhicule
        self.x_vehicle = msg.pose.pose.position.x
        self.y_vehicle = msg.pose.pose.position.y

        # Orientation yaw (2D)
        q = msg.pose.pose.orientation
        self.theta = 2 * atan2(q.z, q.w)  # approximation pour 2D

        # Publier la position du véhicule
        marker = Marker()
        marker.header.frame_id = "odom"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "vehicle"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = self.x_vehicle
        marker.pose.position.y = self.y_vehicle
        marker.pose.position.z = 0.25
        marker.scale.x = 0.5
        marker.scale.y = 0.5
        marker.scale.z = 0.5
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0

        ma = MarkerArray()
        ma.markers.append(marker)
        self.vehicle_pub.publish(ma)

    def cones_callback(self, msg: MarkerArray):
        ma = MarkerArray()
        for m in msg.markers:
            # Transformation en coordonnées globales
            x_global = self.x_vehicle + cos(self.theta) * m.pose.position.x - sin(self.theta) * m.pose.position.y
            y_global = self.y_vehicle + sin(self.theta) * m.pose.position.x + cos(self.theta) * m.pose.position.y

            # Eviter doublons (distance < 0.2m)
            if not any(abs(c[0]-x_global)<0.2 and abs(c[1]-y_global)<0.2 for c in self.global_cones):
                self.global_cones.append((x_global, y_global))

                # Créer un Marker global pour RViz
                marker = Marker()
                marker.header.frame_id = "odom"
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.ns = "cones"
                marker.id = len(self.global_cones)
                marker.type = Marker.CYLINDER
                marker.action = Marker.ADD
                marker.pose.position.x = x_global
                marker.pose.position.y = y_global
                marker.pose.position.z = 0.25
                marker.scale.x = 0.3
                marker.scale.y = 0.3
                marker.scale.z = 0.5
                marker.color.a = 1.0
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
                ma.markers.append(marker)

        self.global_cones_pub.publish(ma)

def main(args=None):
    rclpy.init(args=args)
    node = GlobalCircuit()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
