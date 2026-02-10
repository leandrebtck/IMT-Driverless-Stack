#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import MarkerArray
from nav_msgs.msg import Odometry
import pyqtgraph as pg
from pyqtgraph.Qt import QtWidgets, QtCore

class CircuitMap(Node):
    def __init__(self):
        super().__init__('circuit_map')

        # Subscribers
        self.create_subscription(MarkerArray, '/map/global_cones', self.map_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # Stockage des cônes globaux
        self.global_cones = []  # liste de dicts {'x':.., 'y':..}

        # Position de la voiture
        self.car_x = 0.0
        self.car_y = 0.0

        # ---- Setup PyQtGraph ----
        self.app = QtWidgets.QApplication([])
        self.win = pg.GraphicsLayoutWidget(show=True, title="Carte du circuit (voiture au centre)")
        self.win.resize(900, 900)
        self.plot = self.win.addPlot(title="Vue allocentrique")
        self.plot.setAspectLocked(True)
        self.plot.showGrid(x=True, y=True)

        # Échelle ±1000 m
        self.plot.setXRange(-1000, 1000)
        self.plot.setYRange(-1000, 1000)

        # Scatter plots
        self.cones_scatter = pg.ScatterPlotItem(size=10, pen=pg.mkPen(None), brush=pg.mkBrush(255, 0, 0, 200))
        self.plot.addItem(self.cones_scatter)

        self.car_scatter = pg.ScatterPlotItem(size=15, symbol='t1', pen=pg.mkPen(None), brush=pg.mkBrush(0, 0, 255, 255))
        self.plot.addItem(self.car_scatter)

        # Timer pour rafraîchir PyQtGraph
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_loop)
        self.timer.start(50)  # 20 Hz

        self.get_logger().info("📈 CircuitMap démarré (voiture au centre)")

    def map_callback(self, msg: MarkerArray):
        # Ajouter tous les nouveaux cônes à la liste globale
        for marker in msg.markers:
            x = marker.pose.position.x
            y = marker.pose.position.y
            key = (round(x,2), round(y,2))
            if all((round(c['x'],2), round(c['y'],2)) != key for c in self.global_cones):
                self.global_cones.append({'x': x, 'y': y})
                self.get_logger().info(f"🟢 Cône détecté: X={x:.2f} Y={y:.2f}")

    def odom_callback(self, msg: Odometry):
        self.car_x = msg.pose.pose.position.x
        self.car_y = msg.pose.pose.position.y

    def update_loop(self):
        # Rafraîchit ROS
        rclpy.spin_once(self, timeout_sec=0)

        # Transforme les coordonnées en relatif à la voiture
        if self.global_cones:
            x_rel = [c['x'] - self.car_x for c in self.global_cones]
            y_rel = [c['y'] - self.car_y for c in self.global_cones]
            self.cones_scatter.setData(x_rel, y_rel)

        # La voiture reste au centre (0,0)
        self.car_scatter.setData([0], [0])

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
