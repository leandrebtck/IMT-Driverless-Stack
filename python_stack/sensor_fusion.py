import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from visualization_msgs.msg import Marker, MarkerArray
from vision_msgs.msg import Detection2DArray
from sensor_msgs.msg import CameraInfo, Image
from cv_bridge import CvBridge
import cv2
from tf2_ros import Buffer, TransformListener
from image_geometry import PinholeCameraModel
import tf2_geometry_msgs
from geometry_msgs.msg import PointStamped
import numpy as np
import copy # <--- INDISPENSABLE pour éviter les bugs de référence

class SensorFusionNode(Node):
    def __init__(self):
        super().__init__('sensor_fusion_node')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.bridge = CvBridge()
        self.camera_model = PinholeCameraModel()
        
        # Configuration Caméra 
        self.force_camera_info()
        
        self.latest_image = None
        self.latest_header = None 
        self.latest_detections = None

        self.create_subscription(Image, '/fsds/cam1/image_color', self.image_callback, qos_profile_sensor_data)
        self.create_subscription(MarkerArray, '/lidar/cone_markers', self.lidar_callback, qos_profile_sensor_data)
        self.create_subscription(Detection2DArray, '/perception/detections', self.yolo_callback, qos_profile_sensor_data)

        self.publisher = self.create_publisher(MarkerArray, '/fusion/final_cones', 10)
        self.debug_pub = self.create_publisher(Image, '/fusion/debug_image', 10)
        
        self.get_logger().info("🔍 FUSION V2 (Fix Gray + Time Sync)...")

    def force_camera_info(self):
        width = 416
        height = 416
        f = width / 2.0
        cx = width / 2.0
        cy = height / 2.0
        cam_info = CameraInfo()
        cam_info.width = width
        cam_info.height = height
        cam_info.k = [f, 0.0, cx, 0.0, f, cy, 0.0, 0.0, 1.0]
        cam_info.p = [f, 0.0, cx, 0.0, 0.0, f, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        self.camera_model.fromCameraInfo(cam_info)

    def image_callback(self, msg): 
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            self.latest_header = msg.header
        except Exception: pass

    def yolo_callback(self, msg): 
        self.latest_detections = msg

    def get_bbox_center(self, det):
        # Fonction robuste Iron/Galactic
        try:
            return float(det.bbox.center.x), float(det.bbox.center.y)
        except AttributeError:
            try:
                return float(det.bbox.center.position.x), float(det.bbox.center.position.y)
            except AttributeError:
                return 0.0, 0.0

    def lidar_callback(self, markers_msg):
        if self.latest_image is None: return
        
        try:
            # CORRECTION TEMPORELLE : On demande la transformation "la plus récente possible" (Time 0)
            # rclpy.time.Time() donnait "maintenant", ce qui peut échouer si le TF a 1ms de retard
            transform = self.tf_buffer.lookup_transform(
                'fsds/FSCar', 
                'fsds/Lidar1', 
                rclpy.time.Time(seconds=0) # <--- LE FIX EST ICI
            )
        except Exception:
            return

        debug_img = self.latest_image.copy()
        final_markers = MarkerArray()

        # Dessin des boîtes YOLO pour debug
        if self.latest_detections:
            for det in self.latest_detections.detections:
                cx, cy = self.get_bbox_center(det)
                w, h = float(det.bbox.size_x), float(det.bbox.size_y)
                if cx > 0:
                    # Boîte Verte = YOLO a vu quelque chose
                    cv2.rectangle(debug_img, (int(cx-w/2), int(cy-h/2)), (int(cx+w/2), int(cy+h/2)), (0, 255, 0), 1)

        cones_processed = 0
        
        for marker in markers_msg.markers:
            p_lidar = PointStamped()
            p_lidar.header.frame_id = 'fsds/Lidar1'
            p_lidar.point = marker.pose.position
            
            try:
                p_car = tf2_geometry_msgs.do_transform_point(p_lidar, transform)
                
                # Mathématique de projection (Ajuste ces valeurs si tes points rouges sont décalés)
                rel_x = p_car.point.x - (-0.3)
                rel_y = p_car.point.y - (-0.16)
                rel_z = p_car.point.z - (0.8)

                z_opt = rel_x
                x_opt = -rel_y
                y_opt = -rel_z

                if z_opt > 0.1:
                    u, v = self.camera_model.project3dToPixel((x_opt, y_opt, z_opt))
                    u, v = int(u), int(v)

                    # Point Rouge = Où le Lidar pense que le cône est
                    if 0 <= u < 416 and 0 <= v < 416:
                        cv2.circle(debug_img, (u, v), 5, (0, 0, 255), -1)

                    color_found = False
                    cls_id = -1
                    margin = 80 

                    if self.latest_detections:
                        for det in self.latest_detections.detections:
                            cx, cy = self.get_bbox_center(det)
                            w, h = float(det.bbox.size_x), float(det.bbox.size_y)
                            
                            box_x1 = (cx - w/2) - margin
                            box_x2 = (cx + w/2) + margin
                            box_y1 = (cy - h/2) - margin
                            box_y2 = (cy + h/2) + margin

                            if box_x1 <= u <= box_x2 and box_y1 <= v <= box_y2:
                                if len(det.results) > 0:
                                    cls_id = int(det.results[0].hypothesis.class_id)
                                    color_found = True
                                    break
                    
                    # CORRECTION : On utilise deepcopy pour casser le lien avec l'ancien marker
                    new_marker = copy.deepcopy(marker)
                    new_marker.ns = "fusion_cones"
                    new_marker.color.a = 1.0 # Alpha à 100%
                    
                    if color_found:
                        if cls_id == 0: # Jaune
                            new_marker.color.r, new_marker.color.g, new_marker.color.b = 1.0, 1.0, 0.0
                        elif cls_id == 1: # Bleu
                            new_marker.color.r, new_marker.color.g, new_marker.color.b = 0.0, 0.0, 1.0
                        elif cls_id == 2: # Orange
                            new_marker.color.r, new_marker.color.g, new_marker.color.b = 1.0, 0.5, 0.0
                    else:
                        # NON RECONNU = MAGENTA (Pour bien voir que la fusion a échoué)
                        # Si tu vois ça, c'est que le point rouge n'est pas dans la boîte verte
                        new_marker.color.r, new_marker.color.g, new_marker.color.b = 1.0, 0.0, 1.0
                    
                    final_markers.markers.append(new_marker)
                    cones_processed += 1

            except Exception:
                continue

        self.publisher.publish(final_markers)
        
        # Publication Image Debug
        try:
            debug_msg = self.bridge.cv2_to_imgmsg(debug_img, encoding='bgr8')
            if self.latest_header: debug_msg.header = self.latest_header
            self.debug_pub.publish(debug_msg)
        except: pass

def main(args=None):
    rclpy.init(args=args)
    node = SensorFusionNode()
    try: rclpy.spin(node)
    except: pass
    finally: rclpy.shutdown()

if __name__ == '__main__': main()
