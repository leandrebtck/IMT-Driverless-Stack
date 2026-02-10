#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import MarkerArray
from sensor_msgs.msg import NavSatFix
import pyqtgraph as pg
from pyqtgraph.Qt import QtWidgets, QtCore
import math

def gps_to_xy(lat, lon, ref_lat, ref_lon):
    """Conversion simple GPS -> X/Y mètres (approx equirectangular)."""
    R = 6371000  # rayon terre en m
    x = (lon - ref_lon) * math.cos(math.radians(ref_lat)) * (2 * math.pi * R / 360)
    y = (lat - ref_lat) * (2 * math.pi * R / 360)
    return x, y

class CircuitMap(Node):
    def __init__(self):
        super().__init__('circuit_map')

        # Subscribers
        self.create_subscription(MarkerArray, '/map/global_cones', self.map_callback, 10)
        self.create_subscription(NavSatFix, '/gps', self.gps_callback, 10)

        # Stockage
        self.global_cones = {}  # dictionnaire {id: (x, y)}
        self.car_x = 0.0
        self.car_y = 0.0
        self.ref_lat = None
        self.ref_lon = None

        # ---- Setup PyQtGraph ----
        self.app = QtWidgets.QApplication([])
        self.win = pg.GraphicsLayoutWidget(show=True, title="Carte Globale du Circuit")
        self.win.resize(1000, 1000)
        self.plot = self.win.addPlot(title="Carte 2D (voiture au centre)")
        self.plot.setAspectLocked(True)
        self.plot.showGrid(x=True, y=True)
        self.plot.setXRange(-100, 100)
        self.plot.setYRange(-100, 100)

        # Scatter plots
        self.cones_scatter = pg.ScatterPlotItem(size=10, pen=pg.mkPen(None), brush=pg.mkBrush(255, 0, 0, 200))
        self.plot.addItem(self.cones_scatter)
        self.car_scatter = pg.ScatterPlotItem(size=15, symbol='t1', pen=pg.mkPen(None), brush=pg.mkBrush(0, 0, 255, 255))
        self.plot.addItem(self.car_scatter)

        # Timer
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_loop)
        self.timer.start(50)  # 20 Hz

        self.get_logger().info("📈 CircuitMap démarré (cônes fixes, voiture mobile)")

    def map_callback(self, msg: MarkerArray):
        """Met à jour les coordonnées des cônes globaux."""
        for marker in msg.markers:
            cone_id = marker.id
            x = round(marker.pose.position.x, 2)
            y = round(marker.pose.position.y, 2)
            if cone_id not in self.global_cones:
                self.global_cones[cone_id] = (x, y)
                self.get_logger().info(f"🟢 Cône ajouté ID={cone_id} X={x} Y={y}")

    def gps_callback(self, msg: NavSatFix):
        """Met à jour la position de la voiture selon GPS."""
        if self.ref_lat is None:
            self.ref_lat = msg.latitude
            self.ref_lon = msg.longitude
        self.car_x, self.car_y = gps_to_xy(msg.latitude, msg.longitude, self.ref_lat, self.ref_lon)

    def update_loop(self):
        """Met à jour le graphique."""
        rclpy.spin_once(self, timeout_sec=0)

        # Met à jour les cônes
        if self.global_cones:
            x_list = [c[0] - self.car_x for c in self.global_cones.values()]  # recentrage voiture
            y_list = [c[1] - self.car_y for c in self.global_cones.values()]
            self.cones_scatter.setData(x_list, y_list)

        # Voiture (toujours au centre)
        self.car_scatter.setData([0], [0])

        # Axes dynamiques (fenêtre ±100 m autour de la voiture)
        self.plot.setXRange(-100, 100)
        self.plot.setYRange(-100, 100)

        # Affichage coordonnées dans la console
        if self.global_cones:
            coords = ", ".join([f"({round(c[0],1)}, {round(c[1],1)})" for c in self.global_cones.values()])
            self.get_logger().info(f"📍 Cônes globaux: {coords}")

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
