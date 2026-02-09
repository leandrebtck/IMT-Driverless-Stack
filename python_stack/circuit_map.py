#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import MarkerArray
from nav_msgs.msg import Odometry
from pyqtgraph.Qt import QtWidgets, QtCore
import pyqtgraph as pg
import sys

class CircuitMap(Node):
    def __init__(self):
        super().__init__('circuit_map')

        # --- Subscribers ROS2 ---
        self.create_subscription(MarkerArray, '/cones_relative', self.cones_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # --- Données internes ---
        self.cones_global = set()  # pour éviter les doublons
        self.vehicle_pos = (0.0, 0.0)

        # --- PyQtGraph Setup ---
        self.app = QtWidgets.QApplication([])
        self.win = pg.GraphicsLayoutWidget(title="Circuit Map")
        self.win.show()
        self.win.setWindowTitle('Circuit Map FSDS')
        self.plot = self.win.addPlot()
        self.plot.setAspectLocked(True)  # ratio 1:1
        self.plot.showGrid(x=True, y=True)

        # Scatter plots
        self.cones_plot = pg.ScatterPlotItem(size=10, brush=pg.mkBrush(255,0,0))
        self.vehicle_plot = pg.ScatterPlotItem(size=12, brush=pg.mkBrush(0,0,255))
        self.plot.addItem(self.cones_plot)
        self.plot.addItem(self.vehicle_plot)

        # Timer pour mise à jour graphique
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_plot)
        self.timer.start(50)  # rafraîchit toutes les 50 ms

        self.get_logger().info("✅ CircuitMap lancé et prêt.")

    def cones_callback(self, msg):
        # Ajoute uniquement les nouveaux cônes (en coordonnées globales)
        for m in msg.markers:
            x, y = round(m.pose.position.x,2), round(m.pose.position.y,2)
            self.cones_global.add( (x,y) )

    def odom_callback(self, msg):
        # Position de la voiture
        self.vehicle_pos = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def update_plot(self):
        # Mets à jour la position des cônes
        if self.cones_global:
            cones = list(self.cones_global)
            xs, ys = zip(*cones)
            self.cones_plot.setData(xs, ys)

        # Mets à jour la position de la voiture
        x, y = self.vehicle_pos
        self.vehicle_plot.setData([x], [y])

def main(args=None):
    rclpy.init(args=args)
    node = CircuitMap()
    try:
        # Intègre ROS2 et PyQt dans le même thread
        rclpy_spin_timer = QtCore.QTimer()
        rclpy_spin_timer.timeout.connect(lambda: rclpy.spin_once(node, timeout_sec=0))
        rclpy_spin_timer.start(10)  # spin ROS toutes les 10ms
        sys.exit(node.app.exec_())
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
