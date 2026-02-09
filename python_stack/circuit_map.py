#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import MarkerArray
from nav_msgs.msg import Odometry
import pyqtgraph as pg
from pyqtgraph.Qt import QtWidgets, QtCore
import numpy as np

class CircuitMap(Node):
    def __init__(self):
        super().__init__('circuit_map')

        # Subscribers
        self.create_subscription(MarkerArray, '/map/global_cones', self.map_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # Données
        self.x_cones = []
        self.y_cones = []
        self.car_x = 0.0
        self.car_y = 0.0

        # ---- Setup PyQtGraph ----
        self.app = QtWidgets.QApplication([])
        self.win = pg.GraphicsLayoutWidget(show=True, title="Carte Globale du Circuit")
        self.win.resize(800, 800)
        self.plot = self.win.addPlot(title="Vue Allocentrique (Carte Fixe)")
        self.plot.setAspectLocked(True)
        self.plot.showGrid(x=True, y=True)

        # Cône (Points Rouges)
        self.cones_scatter = pg.ScatterPlotItem(size=10, pen=pg.mkPen(None), brush=pg.mkBrush(255, 100, 0, 255))
        self.plot.addItem(self.cones_scatter)

        # Voiture (Point Bleu)
        self.car_scatter = pg.ScatterPlotItem(size=15, symbol='t1', pen=pg.mkPen(None), brush=pg.mkBrush(0, 0, 255, 255))
        self.plot.addItem(self.car_scatter)

        # Timer IHM (30 Hz)
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_loop)
        self.timer.start(33)

        self.get_logger().info("📈 CircuitMap affichage démarré")

    def map_callback(self, msg: MarkerArray):
        # On met à jour la liste complète des cônes à chaque réception
        # (ConeFusion envoie toute la carte à chaque fois)
        temp_x = []
        temp_y = []
        for marker in msg.markers:
            temp_x.append(marker.pose.position.x)
            temp_y.append(marker.pose.position.y)
        
        self.x_cones = temp_x
        self.y_cones = temp_y

    def odom_callback(self, msg: Odometry):
        self.car_x = msg.pose.pose.position.x
        self.car_y = msg.pose.pose.position.y

    def update_loop(self):
        # Fait tourner ROS un petit coup
        rclpy.spin_once(self, timeout_sec=0)

        # Mise à jour graphique
        # 1. Les cônes (Carte)
        if self.x_cones:
            self.cones_scatter.setData(self.x_cones, self.y_cones)
        
        # 2. La voiture
        self.car_scatter.setData([self.car_x], [self.car_y])

def main(args=None):
    rclpy.init(args=args)
    node = CircuitMap()
    try:
        node.app.exec_()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
