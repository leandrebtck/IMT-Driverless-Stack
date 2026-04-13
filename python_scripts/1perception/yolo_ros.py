#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
import os
import torch

# Messages pour la perception et la fusion
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose

class YoloPerceptionNode(Node):

    def __init__(self):
        super().__init__('yolo_perception_node')

        # ===================================================
        # 1. GESTION DES CHEMINS (Comme ton script qui marche)
        # ===================================================
        script_dir = os.path.dirname(os.path.abspath(__file__))
        
        # On cherche dans le dossier 'weights' le fichier 'best_FINAL.pt'
        weights_path = os.path.join(script_dir, 'weights', 'best_FINAL.pt')

        self.get_logger().info(f"📂 Dossier du script : {script_dir}")
        self.get_logger().info(f"⚖️ Chemin des poids : {weights_path}")

        if not os.path.exists(weights_path):
            self.get_logger().error(f"❌ FICHIER INTROUVABLE : {weights_path}")
            # Petit fallback au cas où il serait à la racine
            if os.path.exists(os.path.join(script_dir, 'best_FINAL.pt')):
                weights_path = os.path.join(script_dir, 'best_FINAL.pt')
                self.get_logger().warn(f"⚠️ Trouvé à la racine ! Utilisation de {weights_path}")
            else:
                raise FileNotFoundError(f"Impossible de trouver best_FINAL.pt dans {weights_path}")

        # ===================================================
        # 2. CHARGEMENT YOLO
        # ===================================================
        self.model = YOLO(weights_path)
        
        # Choix CPU/GPU
        device = 'cuda' if torch.cuda.is_available() else 'cpu'
        self.model.to(device)
        self.get_logger().info(f"🚀 YOLO lancé sur : {device}")

        # ===================================================
        # 3. CONFIGURATION ROS
        # ===================================================
        self.camera_topic = '/fsds/cam1/image_color'
        self.bridge = CvBridge()

        # Subscriber Caméra
        self.subscription = self.create_subscription(
            Image,
            self.camera_topic,
            self.listener_callback,
            qos_profile_sensor_data
        )

        # Publisher Fusion (Detection2DArray est nécessaire pour la fusion)
        self.publisher = self.create_publisher(
            Detection2DArray,
            '/perception/detections',
            10
        )

        # Couleurs et Noms (0=Jaune, 1=Bleu, 2=Orange)
        self.COLORS = { 0: (0, 255, 255), 1: (255, 0, 0), 2: (0, 140, 255) }
        self.CUSTOM_NAMES = { 0: "JAUNE", 1: "BLEU", 2: "ORANGE" }

        self.get_logger().info("✅ Perception lancée (Chemins corrigés + Fix Orange).")

    def listener_callback(self, msg):
        try:
            # 1. Conversion Image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            display_frame = cv_image.copy()

            # 2. Inférence (verbose=False pour ne pas spammer)
            results = self.model(cv_image, verbose=False, conf=0.5)

            # 3. Préparation Message ROS
            detections_msg = Detection2DArray()
            detections_msg.header = msg.header

            if len(results) > 0:
                for box in results[0].boxes:
                    # Données Brutes
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    cls_id = int(box.cls[0])
                    conf = float(box.conf[0])
                    class_name = self.model.names[cls_id].lower()

                    # ===============================================
                    # CORRECTION CLASSE (ORANGE & LARGE)
                    # ===============================================
                    # Si le nom contient "orange" OU "large", c'est un cône orange (ID 2)
                    if "orange" in class_name or "large" in class_name:
                        key_id = 2 
                    elif "blue" in class_name:
                        key_id = 1
                    elif "yellow" in class_name:
                        key_id = 0
                    else:
                        key_id = cls_id # Inconnu

                    # Prépare l'affichage
                    label_text = self.CUSTOM_NAMES.get(key_id, class_name.upper())
                    color = self.COLORS.get(key_id, (0, 255, 0)) # Vert si inconnu
                    full_label = f"{label_text} {conf:.2f}"

                    # Dessin OpenCV
                    cv2.rectangle(display_frame, (x1, y1), (x2, y2), color, 2)
                    (w_text, h_text), _ = cv2.getTextSize(full_label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 1)
                    cv2.rectangle(display_frame, (x1, y1 - 20), (x1 + w_text, y1), color, -1)
                    cv2.putText(display_frame, full_label, (x1, y1 - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,0), 1)

                    # ===============================================
                    # CRÉATION MESSAGE ROS (Compatible Iron & Galactic)
                    # ===============================================
                    w = float(x2 - x1)
                    h = float(y2 - y1)
                    cx = float(x1 + w / 2.0)
                    cy = float(y1 + h / 2.0)

                    detection = Detection2D()
                    detection.header = msg.header
                    
                    # BLOC UNIVERSEL : Essaie .x, sinon .position.x
                    try:
                        detection.bbox.center.x = cx
                        detection.bbox.center.y = cy
                        detection.bbox.center.theta = 0.0
                    except AttributeError:
                        try:
                            detection.bbox.center.position.x = cx
                            detection.bbox.center.position.y = cy
                        except AttributeError:
                            pass # Évite le crash si vraiment tout échoue
                    
                    detection.bbox.size_x = w
                    detection.bbox.size_y = h

                    hyp = ObjectHypothesisWithPose()
                    hyp.hypothesis.class_id = str(key_id) # On envoie l'ID corrigé
                    hyp.hypothesis.score = conf
                    detection.results.append(hyp)
                    
                    detections_msg.detections.append(detection)

            # 4. Publication
            self.publisher.publish(detections_msg)
            
            # 5. Affichage
            cv2.imshow("YOLO FINAL", display_frame)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"Erreur Callback : {e}")


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
