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

# Import nécessaire pour la détection automatique de version
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose, BoundingBox2D

class YoloPerceptionNode(Node):

    def __init__(self):
        super().__init__('yolo_perception_node')

        # ---------------------------------------------------
        # 1. GESTION DES CHEMINS (ABSOLUS)
        # ---------------------------------------------------
        # Récupère le dossier où se trouve CE script (yolo_ros.py)
        script_dir = os.path.dirname(os.path.abspath(__file__))
        
        # Construit le chemin complet vers le fichier de poids
        # ⚠️ Si ton fichier est direct dans le dossier python_stack (pas dans weights), 
        # retire 'weights' ci-dessous : os.path.join(script_dir, 'best.pt')
        weights_path = os.path.join(script_dir, 'weights', 'best.pt')

        self.get_logger().info(f"📂 Dossier script : {script_dir}")
        self.get_logger().info(f"⚖️ Chemin poids  : {weights_path}")

        if not os.path.exists(weights_path):
            self.get_logger().error(f"❌ FICHIER INTROUVABLE : {weights_path}")
            # On tente de le trouver à la racine du script au cas où
            fallback_path = os.path.join(script_dir, 'best.pt')
            if os.path.exists(fallback_path):
                self.get_logger().warn(f"⚠️ Trouvé à la racine ! Utilisation de {fallback_path}")
                weights_path = fallback_path
            else:
                raise FileNotFoundError(f"Impossible de trouver best.pt ni dans {script_dir} ni dans weights/")

        # ---------------------------------------------------
        # 2. CHARGEMENT YOLO
        # ---------------------------------------------------
        self.model = YOLO(weights_path)
        
        # Force le CPU si pas de CUDA, évite les bugs hybrides
        device = 'cuda' if torch.cuda.is_available() else 'cpu'
        self.model.to(device)
        self.get_logger().info(f"🚀 YOLO lancé sur : {device}")

        # ---------------------------------------------------
        # 3. CONFIGURATION ROS
        # ---------------------------------------------------
        self.camera_topic = '/fsds/cam1/image_color'
        self.bridge = CvBridge()

        # Subscriber
        self.subscription = self.create_subscription(
            Image,
            self.camera_topic,
            self.listener_callback,
            qos_profile_sensor_data
        )

        # Publisher
        self.publisher = self.create_publisher(
            Detection2DArray,
            '/perception/detections',
            10
        )

        # ---------------------------------------------------
        # 4. DÉTECTION VERSION ROS (Ta logique intelligente)
        # ---------------------------------------------------
        # On teste si BoundingBox2D a un attribut 'position' (Galactic/Foxy) ou direct x (Iron/Rolling)
        # Note : Parfois Iron utilise aussi position selon les install, donc ce test est le plus sûr.
        try:
            test_bbox = BoundingBox2D()
            # On vérifie si l'objet 'center' (Pose2D) a un attribut 'position'
            if hasattr(test_bbox.center, "position"):
                self.ros_style = "COMPLEX" # Galactic / Humble standard
            else:
                self.ros_style = "SIMPLE"  # Iron / Rolling / Humble modifié
        except Exception:
            self.ros_style = "SIMPLE" # Par défaut

        self.get_logger().info(f"🔧 Mode structure BBox détecté : {self.ros_style}")

        # Couleurs et Noms
        self.COLORS = { 0: (0, 255, 255), 1: (255, 0, 0), 2: (0, 140, 255) }
        self.CUSTOM_NAMES = { 0: "JAUNE", 1: "BLEU", 2: "ORANGE" }


    def listener_callback(self, msg):
        try:
            # 1. Image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            display_frame = cv_image.copy()

            # 2. Inférence
            # verbose=False pour ne pas spammer le terminal
            results = self.model(cv_image, verbose=False, conf=0.5)

            # 3. Message
            detections_msg = Detection2DArray()
            detections_msg.header = msg.header

            if len(results) > 0:
                for box in results[0].boxes:
                    # Données YOLO
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    cls_id = int(box.cls[0])
                    conf = float(box.conf[0])
                    class_name = self.model.names[cls_id].lower()

                    # Gestion des noms et couleurs (Correction Orange)
                    if "orange" in class_name or "large" in class_name:
                        key_id = 2 # Force Orange
                    elif "blue" in class_name:
                        key_id = 1
                    elif "yellow" in class_name:
                        key_id = 0
                    else:
                        key_id = cls_id # Fallback

                    label_text = self.CUSTOM_NAMES.get(key_id, class_name.upper())
                    color = self.COLORS.get(key_id, (0, 255, 0))
                    full_label = f"{label_text} {conf:.2f}"

                    # Dessin
                    cv2.rectangle(display_frame, (x1, y1), (x2, y2), color, 2)
                    (w_text, h_text), _ = cv2.getTextSize(full_label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 1)
                    cv2.rectangle(display_frame, (x1, y1 - 20), (x1 + w_text, y1), color, -1)
                    cv2.putText(display_frame, full_label, (x1, y1 - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,0), 1)

                    # Préparation BBOX ROS
                    w = float(x2 - x1)
                    h = float(y2 - y1)
                    cx = float(x1 + w / 2.0)
                    cy = float(y1 + h / 2.0)

                    detection = Detection2D()
                    detection.header = msg.header
                    
                    # --- APPLICATION DU STYLE DÉTECTÉ AU DÉBUT ---
                    if self.ros_style == "COMPLEX":
                        detection.bbox.center.position.x = cx
                        detection.bbox.center.position.y = cy
                        detection.bbox.center.theta = 0.0
                    else:
                        detection.bbox.center.x = cx
                        detection.bbox.center.y = cy
                        detection.bbox.center.theta = 0.0
                    
                    detection.bbox.size_x = w
                    detection.bbox.size_y = h

                    hyp = ObjectHypothesisWithPose()
                    hyp.hypothesis.class_id = str(key_id) # On envoie l'ID corrigé (0, 1, 2)
                    hyp.hypothesis.score = conf
                    detection.results.append(hyp)
                    
                    detections_msg.detections.append(detection)

            self.publisher.publish(detections_msg)
            
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
