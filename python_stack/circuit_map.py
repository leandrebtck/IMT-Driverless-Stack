#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import MarkerArray
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt
import numpy as np
from math import cos, sin

class CircuitMapGlobal(Node):
    def __init__(self):
        super().__init__('circuit_map_global_node')

        # Stockage des cônes uniques en coordonnées globales
        self.global_cones = []

        # Position et orientation du véhicule
        self.x_vehicle = 0.0
        self.y_vehicle = 0.0
        self.theta = 0.0  # yaw

        # Subscribers
        self.create_subscription(MarkerArray, '/cones_relative', self.cones_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # Setup matplotlib
        plt.ion()
        self.fig, self.ax = plt.subplots()
        self.ax.set_title("Carte globale du circuit")
        self.ax.set_xlabel("X (m)")
        self.ax.set_ylabel("Y (m)")

        self.get_logger().info("Circuit mapping (global) node started")

    def odom_callback(self, msg: Odometry):
        self.x_vehicle = msg.pose.pose.position.x
        self.y_vehicle = msg.pose.pose.position.y

        # Extraction du yaw à partir du quaternion
        q = msg.pose.pose.orientation
        self.theta = np.arctan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))

        # Mise à jour du plot
        self.update_plot()

    def cones_callback(self, msg: MarkerArray):
        for marker in msg.markers:
            # Coordonnées relatives au véhicule
            x_rel = marker.pose.position.x
            y_rel = marker.pose.position.y

            # Transformation en coordonnées globales
            x_global = self.x_vehicle + cos(self.theta)*x_rel - sin(self.theta)*y_rel
            y_global = self.y_vehicle + sin(self.theta)*x_rel + cos(self.theta)*y_rel

            # Ajouter uniquement si nouveau (distance > 0.2 m)
            if not self.is_duplicate(x_global, y_global):
                self.global_cones.append([x_global, y_global])

        self.update_plot()

    def is_duplicate(self, x, y, epsilon=0.2):
        if not self.global_cones:
            return False
        dists = np.linalg.norm(np.array(self.global_cones) - np.array([x,y]), axis=1)
        return np.any(dists < epsilon)

    def update_plot(self):
        self.ax.clear()
        self.ax.set_title("Carte globale du circuit")
        self.ax.set_xlabel("X (m)")
        self.ax.set_ylabel("Y (m)")

        # Affichage des cônes
        if self.global_cones:
            cones = np.array(self.global_cones)
            self.ax.scatter(cones[:,0], cones[:,1], c='red', s=30, label='Cônes')

        # Affichage de la position du véhicule
        self.ax.scatter(self.x_vehicle, self.y_vehicle, c='blue', s=60, label='Voiture')

        self.ax.legend()
        self.ax.axis('equal')  # garde les proportions réelles
        plt.pause(0.001)

def main(args=None):
    rclpy.init(args=args)
    node = CircuitMapGlobal()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        plt.ioff()
        plt.show()

if __name__ == "__main__":
    main()
