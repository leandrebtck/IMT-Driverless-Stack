import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
import os
import torch
import cv2 # Pour l'affichage (optionnel pour sensor)
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose

class YoloPerceptionNode(Node):
    def __init__(self):
        super().__init__('yolo_perception_node')

        # Configuration des poids
        script_dir = os.path.dirname(os.path.abspath(__file__))
        weights_path = os.path.join(script_dir, 'weights', 'best_FINAL.pt')
        
        self.model = YOLO(weights_path)
        self.model.to('cuda' if torch.cuda.is_available() else 'cpu')
        self.bridge = CvBridge()

        # Publisher avec QoS forcée
        self.publisher = self.create_publisher(
            Detection2DArray, 
            '/yolo/detections', 
            qos_profile_sensor_data
        )

        # Subscriber Image
        self.create_subscription(
            Image,
            '/fsds/cam1/image_color',
            self.listener_callback,
            qos_profile_sensor_data
        )
        
        self.get_logger().info("✅ YOLO pret (Topic: /yolo/detections)")

    def listener_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            results = self.model(cv_image, verbose=False, conf=0.5)

            # CONFIGURATION HEADLESS (AFFICHAGE VIDÉO)
            # Enlèver les '#' ci-dessous pour voir la fenêtre (déconseillé si vous voulez lancer sensor)
            # Laisse les '#' pour la performance (ex : sensor fusion)
            
            annotated_frame = results[0].plot()        # <----
            cv2.imshow("YOLO DEBUG VIEW", annotated_frame)   # <----
            cv2.waitKey(1)     # <----
            # ====================================================

            detections_msg = Detection2DArray()
            detections_msg.header = msg.header 

            for box in results[0].boxes:
                # Calcul des coordonnées de la boîte
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                cls_id = int(box.cls[0])
                
                detection = Detection2D()
                detection.bbox.center.x = float(x1 + (x2 - x1) / 2.0)
                detection.bbox.center.y = float(y1 + (y2 - y1) / 2.0)
                detection.bbox.size_x = float(x2 - x1)
                detection.bbox.size_y = float(y2 - y1)

                hyp = ObjectHypothesisWithPose()
                hyp.hypothesis.class_id = str(cls_id)
                hyp.hypothesis.score = float(box.conf[0])
                detection.results.append(hyp)
                detections_msg.detections.append(detection)

            # Envoi et log de debug
            self.publisher.publish(detections_msg)
            if len(detections_msg.detections) > 0:
                print(f"DEBUG YOLO : {len(detections_msg.detections)} cones detectes")

        except Exception as e:
            print(f"Erreur YOLO : {e}")

def main(args=None):
    rclpy.init(args=args)
    node = YoloPerceptionNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
