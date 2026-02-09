#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import MarkerArray
import matplotlib.pyplot as plt
import numpy as np

class CircuitMap(Node):
    def __init__(self):
        super().__init__('circuit_map_node')

        # Liste pour stocker les cônes uniques
        self.all_cones = np.empty((0,2))

        # Subscriber
        self.create_subscription(
            MarkerArray,
            '/cones_relative',
            self.cones_callback,
            10
        )

        # Setup matplotlib
        plt.ion()
        self.fig, self.ax = plt.subplots()
        self.ax.set_title("Carte du circuit (cônes détectés)")
        self.ax.set_xlabel("X (m)")
        self.ax.set_ylabel("Y (m)")

        self.get_logger().info("Circuit mapping node started")

    def cones_callback(self, msg):
        new_cones = []

        for marker in msg.markers:
            x = marker.pose.position.x
            y = marker.pose.position.y
            new_cones.append([x, y])

        if new_cones:
            new_cones = np.array(new_cones)

            # Ajouter seulement les cônes nouveaux (distance > 0.2 m)
            for pt in new_cones:
                if self.all_cones.shape[0] == 0:
                    self.all_cones = np.vstack([self.all_cones, pt])
                else:
                    dists = np.linalg.norm(self.all_cones - pt, axis=1)
                    if np.all(dists > 0.2):  # epsilon = 20 cm
                        self.all_cones = np.vstack([self.all_cones, pt])

            # Mise à jour du plot
            self.update_plot()

    def update_plot(self):
        self.ax.clear()
        self.ax.set_title("Carte du circuit (cônes détectés)")
        self.ax.set_xlabel("X (m)")
        self.ax.set_ylabel("Y (m)")

        if self.all_cones.shape[0] > 0:
            self.ax.scatter(self.all_cones[:,0], self.all_cones[:,1], c='red', s=50, label='Cônes')
            self.ax.legend()

        plt.pause(0.001)

def main(args=None):
    rclpy.init(args=args)
    node = CircuitMap()
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
