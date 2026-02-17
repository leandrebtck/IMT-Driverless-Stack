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

        self.get_logger().info(f"📍 Dossier du script : {script_dir}")
        
        # Vérification 
        if not os.path.exists(weights_path):
            self.get_logger().error(f" FICHIER INTROUVABLE ! Vérifier que le dossier 'weights' est bien dans {script_dir}")
        else:
            self.get_logger().info("✅ Poids trouvés ! Chargement...")

        self.model_path = weights_path
        self.camera_topic = '/fsds/cam1/image_color'

        # --- CORRECTION COULEURS (BGR) ---
        # OpenCV utilise BGR (Blue, Green, Red)
        # J'ai inversé : 0 est devenu Jaune, 1 est devenu Bleu
        self.COLORS = {
            0: (0, 255, 255), # JAUNE (B=0, G=255, R=255) -> Correction
            1: (255, 0, 0),   # BLEU  (B=255, G=0, R=0)   -> Correction
            2: (0, 0, 255),   # ROUGE (Orange)
        }

        # Noms personnalisés (Optionnel, pour l'affichage texte)
        self.CUSTOM_NAMES = {
            0: "JAUNE",
            1: "BLEU",
            2: "ORANGE",
        }

        try:
            self.model = YOLO(self.model_path)
        except Exception as e:
            self.get_logger().error(f" Erreur modèle : {e}")
            raise e

        # MODE INFÉRENCE
        self.model.eval()
        device = 'cuda' if torch.cuda.is_available() else 'cpu'
        self.model.to(device)
        self.get_logger().info(f"🚀 YOLO lancé sur : {device}")

        self.bridge = CvBridge()

        # Subscriber Image
        self.subscription = self.create_subscription(
            Image,
            self.camera_topic,
            self.listener_callback,
            qos_profile_sensor_data
        )

        # Publisher pour la Fusion
        self.publisher = self.create_publisher(
            Detection2DArray, 
            '/yolo/detections', 
            10
        )

        self.get_logger().info("✅ Perception lancée avec Couleurs CORRIGÉES.")

    def listener_callback(self, msg):
        try:
            # 1. Réception
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            display_frame = cv_image.copy()

            # 2. Inférence
            with torch.no_grad():
                results = self.model(cv_image, verbose=False, conf=0.5)

            # Préparation du message global
            detections_msg = Detection2DArray()
            detections_msg.header = msg.header 
    
            # 3. Traitement
            for box in results[0].boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                cls_id = int(box.cls[0])
                conf = float(box.conf[0])

                # Affichage Visuel
                name_display = self.CUSTOM_NAMES.get(cls_id, str(cls_id))
                label = f"{name_display} {conf:.2f}"
                
                # Récupération de la couleur corrigée
                color = self.COLORS.get(cls_id, (255, 255, 255))

                # Dessin sur l'image de debug
                cv2.rectangle(display_frame, (x1, y1), (x2, y2), color, 2)
                cv2.putText(display_frame, label, (x1, y1 - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

                # --- PARTIE FUSION (ROS MESSAGE) ---
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

            # 4. Envoi ROS
            self.publisher.publish(detections_msg)
            
            # 5. Affichage Fenêtre
            # Mettre des # ci-dessous pour le mode HEADLESS (Performance)
            cv2.imshow("YOLO FINAL", display_frame)  <-- # ICI
            cv2.waitKey(1)                           <-- # ICI

        except Exception as e:
            self.get_logger().error(f"❌ Erreur Display : {e}")

def main(args=None):
    rclpy.init(args=args)
    node = YoloPerceptionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
