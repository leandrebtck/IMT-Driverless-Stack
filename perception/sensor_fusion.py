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
import copy 

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
        
        self.get_logger().info("🔍 FUSION TF2 PURE (Zéro offset manuel)...")

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
            # === CORRECTION MAJEURE ===
            # On ne demande plus la transformation vers la VOITURE ('fsds/FSCar')
            # Mais directement vers la CAMÉRA ('fsds/cam1')
            # ROS s'occupe de tous les décalages (x, y, z) automatiquement.
            transform = self.tf_buffer.lookup_transform(
                'fsds/cam1',       # CIBLE : La caméra
                'fsds/Lidar1',     # SOURCE : Le Lidar
                rclpy.time.Time(seconds=0)
            )
        except Exception as e:
            # On log moins souvent pour ne pas spammer si le TF n'est pas prêt
            self.get_logger().warning(f"TF non prêt: {e}", throttle_duration_sec=2)
            return

        debug_img = self.latest_image.copy()
        final_markers = MarkerArray()

        # Dessin carrés verts (YOLO)
        if self.latest_detections:
            for det in self.latest_detections.detections:
                cx, cy = self.get_bbox_center(det)
                w, h = float(det.bbox.size_x), float(det.bbox.size_y)
                if cx > 0:
                    cv2.rectangle(debug_img, (int(cx-w/2), int(cy-h/2)), (int(cx+w/2), int(cy+h/2)), (0, 255, 0), 1)

        cones_processed = 0
        
        for marker in markers_msg.markers:
            p_lidar = PointStamped()
            p_lidar.header.frame_id = 'fsds/Lidar1'
            p_lidar.point = marker.pose.position
            
            try:
                # 1. TRANSFORMATION AUTOMATIQUE (Grâce au TF)
                # p_cam contient maintenant le point VU DEPUIS LA CAMÉRA
                # Mais attention : Dans le repère "Physique" de la caméra (X=Devant, Y=Gauche, Z=Haut)
                p_cam = tf2_geometry_msgs.do_transform_point(p_lidar, transform)
                
                # 2. PASSAGE EN REPÈRE OPTIQUE (Nécessaire pour project3dToPixel)
                # Une caméra "Optique" voit : Z=Profondeur, X=Droite, Y=Bas
                # On fait juste la rotation des axes standard, SANS ajouter de décalage manuel (genre -0.3)
                
                x_phys = p_cam.point.x
                y_phys = p_cam.point.y
                z_phys = p_cam.point.z

                # Conversion Standard ROS Body -> Optical
                z_opt = x_phys      # La profondeur (Z) c'est ce qui est devant (X)
                x_opt = -y_phys     # La droite (X) c'est l'inverse de la gauche (Y)
                y_opt = -z_phys     # Le bas (Y) c'est l'inverse du haut (Z)

                # Si le point est devant la caméra (Z > 0.1m)
                if z_opt > 0.1:
                    u, v = self.camera_model.project3dToPixel((x_opt, y_opt, z_opt))
                    u, v = int(u), int(v)

                    # Dessin Point Rouge
                    if 0 <= u < 416 and 0 <= v < 416:
                        cv2.circle(debug_img, (u, v), 5, (0, 0, 255), -1)

                    color_found = False
                    cls_id = -1
                    margin = 80 

                    if self.latest_detections:
                        for det in self.latest_detections.detections:
                            cx, cy = self.get_bbox_center(det)
                            w, h = float(det.bbox.size_x), float(det.bbox.size_y)
                            
                            # Test d'inclusion
                            if (cx - w/2 - margin) <= u <= (cx + w/2 + margin) and \
                               (cy - h/2 - margin) <= v <= (cy + h/2 + margin):
                                if len(det.results) > 0:
                                    cls_id = int(det.results[0].hypothesis.class_id)
                                    color_found = True
                                    break
                    
                    new_marker = copy.deepcopy(marker)
                    new_marker.ns = "fusion_cones"
                    new_marker.color.a = 1.0
                    
                    if color_found:
                        if cls_id == 0: # Jaune
                            new_marker.color.r, new_marker.color.g, new_marker.color.b = 1.0, 1.0, 0.0
                        elif cls_id == 1: # Bleu
                            new_marker.color.r, new_marker.color.g, new_marker.color.b = 0.0, 0.0, 1.0
                        elif cls_id == 2: # Orange
                            new_marker.color.r, new_marker.color.g, new_marker.color.b = 1.0, 0.5, 0.0
                    else:
                        # Magenta pour les erreurs de fusion
                        new_marker.color.r, new_marker.color.g, new_marker.color.b = 1.0, 0.0, 1.0
                    
                    final_markers.markers.append(new_marker)
                    cones_processed += 1

            except Exception:
                continue

        self.publisher.publish(final_markers)
        
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
