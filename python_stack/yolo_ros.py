import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
import os
import sys
import torch
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose

class YoloPerceptionNode(Node):
    def __init__(self):
        super().__init__('yolo_perception_node')

        # --- CONFIGURATION ---
        script_dir = os.path.dirname(os.path.abspath(__file__))
        weights_path = os.path.join(script_dir, 'weights', 'best_FINAL.pt')

        if not os.path.exists(weights_path):
            self.get_logger().error(f"❌ Poids introuvables : {weights_path}")
        else:
            self.get_logger().info(f"✅ Chargement : {weights_path}")

        self.model = YOLO(weights_path)
        self.model.eval()
        
        # Accélération GPU si possible
        device = 'cuda' if torch.cuda.is_available() else 'cpu'
        self.model.to(device)
        self.get_logger().info(f"YOLO tourne sur : {device}")

        self.bridge = CvBridge()

        # Abonnement Caméra (Mode QoS Rapide)
        self.subscription = self.create_subscription(
            Image,
            '/fsds/cam1/image_color',
            self.listener_callback,
            qos_profile_sensor_data
        )

        # Publisher pour la Fusion
        self.publisher = self.create_publisher(
            Detection2DArray, 
            '/yolo/detections', 
            10
        )
        
        self.get_logger().info("YOLO en mode headless")

    def listener_callback(self, msg):
        try:
            # 1. Conversion
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # 2. Inférence (Sans affichage = Plus rapide)
            with torch.no_grad():
                results = self.model(cv_image, verbose=False, conf=0.5)

            # 3. Préparation du message ROS
            detections_msg = Detection2DArray()
            detections_msg.header = msg.header 

            if len(results) > 0:
                for box in results[0].boxes:
                    # Coordonnées
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    cls_id = int(box.cls[0])
                    conf = float(box.conf[0])

                    # Conversion pour ROS (Centre + Taille)
                    w = float(x2 - x1)
                    h = float(y2 - y1)
                    cx = float(x1 + w / 2.0)
                    cy = float(y1 + h / 2.0)

                    detection = Detection2D()
                    detection.header = msg.header
                    
                    detection.bbox.center.x = cx
                    detection.bbox.center.y = cy
                    detection.bbox.size_x = w
                    detection.bbox.size_y = h

                    hypothesis = ObjectHypothesisWithPose()
                    hypothesis.hypothesis.class_id = str(cls_id)
                    hypothesis.hypothesis.score = conf
                    detection.results.append(hypothesis)
                    
                    detections_msg.detections.append(detection)

            # 4. Publication (C'est le plus important)
            self.publisher.publish(detections_msg)

            #ON A SUPPRIMÉ cv2.imshow ET cv2.waitKey (car dès que l'affichage plantait, yolo perception ne publiait plus rien)

        except Exception as e:
            self.get_logger().error(f"❌ Erreur : {e}")

def main(args=None):
    rclpy.init(args=args)
    node = YoloPerceptionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
