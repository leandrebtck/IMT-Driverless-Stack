#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import MarkerArray
import matplotlib.pyplot as plt

class CircuitMap(Node):
    def __init__(self):
        super().__init__('circuit_map_node')

        # Liste pour stocker tous les cônes détectés
        self.all_cones = []

        # Subscriber
        self.create_subscription(
            MarkerArray,
            '/cones_relative',
            self.cones_callback,
            10
        )

        # Setup matplotlib
        plt.ion()  # mode interactif
        self.fig, self.ax = plt.subplots()
        self.ax.set_title("Carte du circuit (cônes détectés)")
        self.ax.set_xlabel("X (m)")
        self.ax.set_ylabel("Y (m)")
        self.scatter = None

        self.get_logger().info("Circuit mapping node started")

    def cones_callback(self, msg):
        # Ajouter les coordonnées de tous les cônes reçus
        for marker in msg.markers:
            x = marker.pose.position.x
            y = marker.pose.position.y
            self.all_cones.append((x, y))

        # Mettre à jour le plot
        self.update_plot()

    def update_plot(self):
        self.ax.clear()
        self.ax.set_title("Carte du circuit (cônes détectés)")
        self.ax.set_xlabel("X (m)")
        self.ax.set_ylabel("Y (m)")

        if self.all_cones:
            xs, ys = zip(*self.all_cones)
            self.ax.scatter(xs, ys, c='red', s=50, label='Cônes')
            self.ax.legend()
        plt.pause(0.001)  # pause courte pour rafraîchir le plot

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
