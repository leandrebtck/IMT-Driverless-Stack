#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import MarkerArray
from nav_msgs.msg import Odometry
from pyqtgraph.Qt import QtWidgets, QtCore
import pyqtgraph as pg
import sys
import math

class CircuitMap(Node):
    def __init__(self):
        super().__init__('circuit_map')

        # ROS2 Subscribers
        self.create_subscription(MarkerArray, '/cones_relative', self.cones_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # Data
        self.cones_global = []  # liste des cônes (x,y)
        self.vehicle_pos = (0.0, 0.0)

        # PyQtGraph
        self.app = QtWidgets.QApplication([])
        self.win = pg.GraphicsLayoutWidget(title="Circuit Map")
        self.win.show()
        self.plot = self.win.addPlot()
        self.plot.setAspectLocked(True)
        self.plot.showGrid(x=True, y=True)

        # Scatter plots
        self.cones_plot = pg.ScatterPlotItem(size=10, brush=pg.mkBrush(255,0,0))
        self.vehicle_plot = pg.ScatterPlotItem(size=12, brush=pg.mkBrush(0,0,255))
        self.plot.addItem(self.cones_plot)
        self.plot.addItem(self.vehicle_plot)

        # Ligne pour relier 2 cônes
        self.line_item = pg.PlotDataItem(pen=pg.mkPen(200,0,0, width=1))
        self.plot.addItem(self.line_item)

        # Timer pour mise à jour
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_plot)
        self.timer.start(50)

        self.get_logger().info("✅ CircuitMap lancé.")

    def cones_callback(self, msg):
        for m in msg.markers:
            x, y = m.pose.position.x, m.pose.position.y
            # On ajoute uniquement si nouveau cône (distance > 0.1 m des existants)
            if all(math.hypot(x-cx, y-cy) > 0.1 for cx, cy in self.cones_global):
                self.cones_global.append((x,y))

    def odom_callback(self, msg):
        self.vehicle_pos = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def update_plot(self):
        # Affiche la voiture
        x, y = self.vehicle_pos
        self.vehicle_plot.setData([x],[y])

        if len(self.cones_global) == 0:
            return

        # Calculer les distances par rapport à la voiture
        distances = [(math.hypot(cx - x, cy - y), (cx, cy)) for cx, cy in self.cones_global]
        distances.sort(key=lambda t: t[0])

        # Prendre les 2 cônes les plus proches
        closest = [pt for _, pt in distances[:2]]

        if len(closest) == 1:
            # un seul cône proche
            self.cones_plot.setData([closest[0][0]], [closest[0][1]])
            self.line_item.setData([],[])
        elif len(closest) == 2:
            # deux cônes proches : points + ligne
            xs = [closest[0][0], closest[1][0]]
            ys = [closest[0][1], closest[1][1]]
            self.cones_plot.setData(xs, ys)
            self.line_item.setData(xs, ys)

def main(args=None):
    rclpy.init(args=args)
    node = CircuitMap()
    try:
        spin_timer = QtCore.QTimer()
        spin_timer.timeout.connect(lambda: rclpy.spin_once(node, timeout_sec=0))
        spin_timer.start(10)
        sys.exit(node.app.exec_())
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
