#!/usr/bin/env python3

import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from config_loader import CFG

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
import message_filters
from ultralytics import YOLO
import torch
import threading

from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose

# ── Paramètres chargés depuis config.yaml ────────────────────────────────────
_cam = CFG['camera']
FSG_CONE_REAL_HEIGHT_M = _cam['cone_real_height_m']
BASELINE_M             = _cam['baseline_m']
FOCAL_PX               = _cam['focal_px']
DEPTH_MAX_AGE_S        = _cam['depth_max_age_s']
CY_TOLERANCE_PX        = _cam['epipolar_tolerance_px']
DISP_MIN_BBOX_PX       = _cam['disparity_min_px']
DISP_MAX_BBOX_PX       = _cam['disparity_max_px']
Z_STEREO_MIN_M         = _cam['stereo_min_depth_m']
Z_STEREO_MAX_M         = _cam['stereo_max_depth_m']
DEPTH_OFFSET           = {int(k): v for k, v in _cam['depth_offset_by_class'].items()}

_topics = CFG['topics']
_frames = CFG['frames']


class YoloStereoNode(Node):
    """
    Node ROS 2 de perception visuelle pour Formula Student Driverless.

    Pipeline de detection :
      1. Inference YOLO simultanee sur les images gauche et droite (batch).
      2. Construction de deux listes de cones detectes (gauche / droite).
      3. Matching stéréo par bounding box :
         - Contrainte epipolaire : meme ligne horizontale (cy similaire)
         - Contrainte geometrique : disparite positive (cx_gauche > cx_droite)
         - La camera gauche voit l'objet plus a droite que la camera droite
         - Profondeur : Z = f * B / (cx_gauche - cx_droite)
         - Intervalle arbritraire: 2m a 12m
      4. Fallback monoculaire (taille apparente du cone) au-dela de 12m
         ou en l'absence de correspondance.
      5. Correction de classe par analyse HSV pour les faux positifs orange.
      6. Publication sur /perception/stereo_detections.
    """

    def __init__(self):
        super().__init__('yolo_stereo_node')
        self.bridge = CvBridge()

        self.baseline     = BASELINE_M
        self.focal_length = FOCAL_PX
        self.fx           = None

        self.latest_depth       = None
        self.latest_depth_stamp = None

        self._display_frame = None
        self._display_lock  = threading.Lock()

        # Compteurs de methodes pour monitoring en production
        self._method_counts = {'STEREO_BBOX': 0, 'MONO': 0, 'NONE': 0}
        self._log_interval  = 100
        self._total_dets    = 0

        script_dir   = os.path.dirname(os.path.abspath(__file__))
        weights_path = os.path.join(script_dir, 'weights', 'best_FINAL.pt')

        self.get_logger().info(f"Dossier du script : {script_dir}")

        if not os.path.exists(weights_path):
            if os.path.exists(os.path.join(script_dir, 'best_FINAL.pt')):
                weights_path = os.path.join(script_dir, 'best_FINAL.pt')
            else:
                raise FileNotFoundError("Impossible de trouver best_FINAL.pt")

        self.model = YOLO(weights_path)
        device = 'cuda' if torch.cuda.is_available() else 'cpu'
        self.model.to(device)
        self.get_logger().info(f"YOLO lance sur : {device}")

        self.COLORS       = {0: (0, 255, 255), 1: (255, 0, 0), 2: (0, 140, 255)}
        self.CUSTOM_NAMES = {0: "JAUNE", 1: "BLEU", 2: "ORANGE"}

        self.sub_cam_info = self.create_subscription(
            CameraInfo, _topics['camera_info_left'], self.camera_info_cb, 10
        )

        # Synchronisation temporelle stricte entre cam1 et cam2 (tolerance 50ms)
        self.sub_left = message_filters.Subscriber(
            self, Image, _topics['camera_left'],
            qos_profile=qos_profile_sensor_data
        )
        self.sub_right = message_filters.Subscriber(
            self, Image, _topics['camera_right'],
            qos_profile=qos_profile_sensor_data
        )
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.sub_left, self.sub_right], 10, 0.05
        )
        self.ts.registerCallback(self.sync_callback)

        self.publisher = self.create_publisher(
            Detection2DArray, _topics['stereo_detections'], 10
        )

        self.get_logger().info("Perception Stereo BBox lancee.")
        self.get_logger().info(
            f"Parametres matching : "
            f"CY_TOL={CY_TOLERANCE_PX}px | "
            f"DISP=[{DISP_MIN_BBOX_PX},{DISP_MAX_BBOX_PX}]px | "
            f"Z=[{Z_STEREO_MIN_M},{Z_STEREO_MAX_M}]m"
        )

    # ------------------------------------------------------------------
    # Camera info
    # ------------------------------------------------------------------

    def camera_info_cb(self, msg):
        """Recupere fx depuis la matrice intrinseque K (K[0] = fx)."""
        if self.fx is None:
            self.fx           = msg.k[0]
            self.focal_length = self.fx
            self.get_logger().info(f"Focale recue : {self.focal_length:.2f} px")

    # ------------------------------------------------------------------
    # Monitoring
    # ------------------------------------------------------------------

    def _log_method_stats(self):
        """Log la distribution des methodes d'estimation toutes les
        _log_interval detections."""
        self._total_dets += 1
        if self._total_dets % self._log_interval == 0:
            total = sum(self._method_counts.values())
            if total > 0:
                stats = " | ".join(
                    f"{m}: {c} ({100*c//total}%)"
                    for m, c in self._method_counts.items()
                    if c > 0
                )
                self.get_logger().info(f"[METHODES sur {total} dets] {stats}")

    # ------------------------------------------------------------------
    # Construction de la liste de cones detectes
    # ------------------------------------------------------------------

    def _build_cone_list(self, yolo_result, img_bgr):
        """
        Construit la liste des cones detectes a partir d'un resultat YOLO.

        Chaque element est un dict :
          cx, cy  : centre de la bounding box en pixels
          x1,y1,x2,y2 : coordonnees de la bbox
          key_id  : classe FSG (0=jaune, 1=bleu, 2=orange)
          conf    : confiance YOLO
          h_box   : hauteur de la bbox en pixels

        La classe est eventuellement corrigee par analyse HSV
        pour les faux positifs orange (eclairage chaud).
        """
        cones = []
        if yolo_result is None:
            return cones

        for box in yolo_result.boxes:
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            cls_id     = int(box.cls[0])
            conf       = float(box.conf[0])
            class_name = self.model.names[cls_id].lower()

            if "orange" in class_name or "large" in class_name:
                key_id = 2
            elif "blue" in class_name:
                key_id = 1
            elif "yellow" in class_name:
                key_id = 0
            else:
                key_id = cls_id

            key_id = self.validate_class_by_color(img_bgr, x1, y1, x2, y2, key_id)

            w_box = x2 - x1
            h_box = y2 - y1
            cx    = x1 + w_box / 2.0
            cy    = y1 + h_box / 2.0

            cones.append({
                'cx': cx, 'cy': cy,
                'x1': x1, 'y1': y1, 'x2': x2, 'y2': y2,
                'key_id': key_id, 'conf': conf,
                'w_box': w_box, 'h_box': h_box,
            })

        return cones

    # ------------------------------------------------------------------
    # Matching stéréo par bounding box
    # ------------------------------------------------------------------

    def match_stereo_bboxes(self, cones_left, cones_right):
        """
        Associe chaque cone de l'image gauche a son correspondant dans l'image droite.

        Principe geometrique :
          - Les deux cameras sont alignees horizontalement (meme Y dans settings.json).
          - Un cone apparait a la meme hauteur (cy) dans les deux images :
            contrainte epipolaire horizontale.
          - La camera gauche (cam1, Y=-0.32) voit le cone plus a droite que la camera
            droite (cam2, Y=+0.32) : disparite d = cx_gauche - cx_droite > 0.
          - Profondeur : Z = focale * baseline / disparite.

        Criteres de matching pour une paire (cone_gauche, cone_droite) :
          1. Meme classe (key_id identique).
          2. |cy_gauche - cy_droite| <= CY_TOLERANCE_PX (meme ligne horizontale).
          3. DISP_MIN_BBOX_PX <= (cx_gauche - cx_droite) <= DISP_MAX_BBOX_PX.
          4. Z_STEREO_MIN_M <= Z <= Z_STEREO_MAX_M.

        En cas de plusieurs candidats dans l'image droite pour un meme cone gauche,
        on retient celui dont le cy est le plus proche (meilleure coherence epipolaire).

        Retourne une liste de tuples (cone_gauche, z_estime).
        """
        matched = []

        for cone_l in cones_left:
            best_candidate = None
            best_cy_diff   = float('inf')

            for cone_r in cones_right:

                # Critere 1 : meme classe
                if cone_l['key_id'] != cone_r['key_id']:
                    continue

                # Critere 2 : contrainte epipolaire (meme ligne)
                cy_diff = abs(cone_l['cy'] - cone_r['cy'])
                if cy_diff > CY_TOLERANCE_PX:
                    continue

                # Critere 3 : disparite dans l'intervalle valide
                # cx_gauche > cx_droite pour un objet devant le vehicule
                disp = cone_l['cx'] - cone_r['cx']
                if not (DISP_MIN_BBOX_PX <= disp <= DISP_MAX_BBOX_PX):
                    continue

                # Critere 4 : profondeur dans l'intervalle fiable
                z = (self.focal_length * self.baseline) / disp
                if not (Z_STEREO_MIN_M <= z <= Z_STEREO_MAX_M):
                    continue

                # Parmi les candidats valides, on retient le plus proche en cy
                if cy_diff < best_cy_diff:
                    best_cy_diff   = cy_diff
                    best_candidate = (cone_r, z, disp)

            if best_candidate is not None:
                cone_r, z, disp = best_candidate
                matched.append((cone_l, z, disp))
            else:
                # Aucune correspondance trouvee : fallback monoculaire
                z_mono, _ = self.estimate_depth_monocular(cone_l['h_box'])
                matched.append((cone_l, z_mono, None))

        return matched

    # ------------------------------------------------------------------
    # Estimation monoculaire (fallback)
    # ------------------------------------------------------------------

    def estimate_depth_monocular(self, bbox_height_px):
        """
        Estimation de profondeur par taille apparente du cone.
        Formule : Z = f * H_reel / H_px
        Valable uniquement si la hauteur reelle du cone est connue (FSG = 0.325m).
        Precision limitee par la qualite de la bbox et la perspective.
        """
        if bbox_height_px < 5:
            return None, None
        return (self.focal_length * FSG_CONE_REAL_HEIGHT_M) / bbox_height_px, 'MONO'

    # ------------------------------------------------------------------
    # Validation de classe par couleur HSV
    # ------------------------------------------------------------------

    def validate_class_by_color(self, img_bgr, x1, y1, x2, y2, key_id):
        """
        Verifie la coherence entre la classe YOLO et la couleur mesuree en HSV.
        Corrige les faux positifs orange causes par un eclairage chaud.
        La correction est conservative : elle s'applique uniquement si la classe
        predite est orange et que la couleur dominante mesuree est une autre classe
        avec un ratio superieur a 8% des pixels du ROI.
        """
        roi = img_bgr[max(0, y1):min(img_bgr.shape[0], y2),
                      max(0, x1):min(img_bgr.shape[1], x2)]
        if roi.size == 0:
            return key_id

        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        # Plages HSV des cones FSG sous eclairage standard
        # Saturation minimale de 80 pour exclure les zones grises ou surexposees
        mask_yellow = cv2.inRange(hsv, (20,  80,  80), (35,  255, 255))
        mask_blue   = cv2.inRange(hsv, (100, 80,  60), (130, 255, 255))
        mask_orange = cv2.inRange(hsv, (5,   120, 80), (20,  255, 255))

        counts = {
            0: int(cv2.countNonZero(mask_yellow)),
            1: int(cv2.countNonZero(mask_blue)),
            2: int(cv2.countNonZero(mask_orange)),
        }
        total_pixels = roi.shape[0] * roi.shape[1]
        best_id      = max(counts, key=counts.get)
        ratio        = counts[best_id] / max(total_pixels, 1)

        if key_id == 2 and best_id != 2 and ratio > 0.08:
            self.get_logger().debug(
                f"Correction couleur : ORANGE -> {self.CUSTOM_NAMES[best_id]} "
                f"(ratio={ratio:.2f})"
            )
            return best_id

        return key_id

    # ------------------------------------------------------------------
    # Score de confiance
    # ------------------------------------------------------------------

    def compute_confidence(self, method, z, disp=None):
        """
        Calcule un score de confiance 0-100% selon la methode utilisee.

        STEREO_BBOX : base 80%, bonifiee par la proximite de l'objet.
                      La disparite elevee (objet proche) donne un meilleur matching.
        MONO        : proportionnel a la taille de la bbox.
                      Grande bbox = cone proche = estimation plus precise.
        """
        if method is None or z is None or z <= 0:
            return 0.0

        if method == 'STEREO_BBOX':
            z_score = max(0.0, (Z_STEREO_MAX_M - z) / Z_STEREO_MAX_M)
            conf    = (0.80 + 0.20 * z_score) * 100.0

        elif method == 'MONO':
            bbox_h_est = (self.focal_length * FSG_CONE_REAL_HEIGHT_M) / z
            conf       = min(bbox_h_est / 30.0, 1.0) * 100.0

        else:
            conf = 0.0

        return round(conf, 1)

    # ------------------------------------------------------------------
    # Callback principal
    # ------------------------------------------------------------------

    def sync_callback(self, msg_left, msg_right):
        try:
            img_left  = self.bridge.imgmsg_to_cv2(msg_left,  "bgr8")
            img_right = self.bridge.imgmsg_to_cv2(msg_right, "bgr8")

            display_frame = img_left.copy()

            # Inference YOLO en batch sur les deux images simultanement.
            # Plus efficace qu'une double inference sequentielle sur GPU.
            with torch.no_grad():
                results = self.model([img_left, img_right], verbose=False, conf=0.5)

            # Construction des listes de cones detectes par chaque camera
            cones_left  = self._build_cone_list(results[0], img_left)
            cones_right = self._build_cone_list(results[1], img_right)

            # Matching stéréo : associe chaque cone gauche a son correspondant droit
            # et calcule la profondeur par disparite ou par fallback monoculaire
            matched_cones = self.match_stereo_bboxes(cones_left, cones_right)

            detections_msg        = Detection2DArray()
            detections_msg.header = msg_left.header

            for cone_l, z_obj, disp in matched_cones:

                method = 'STEREO_BBOX' if disp is not None else 'MONO'

                # Application de la correction empirique de biais uniquement
                # sur les mesures stereo (le biais monoculaire est de nature differente)
                if z_obj is not None and method == 'STEREO_BBOX':
                    z_obj = max(0.1, z_obj + DEPTH_OFFSET.get(cone_l['key_id'], 0.0))

                depth_conf = self.compute_confidence(method, z_obj, disp)

                # Mise a jour des compteurs de monitoring
                self._method_counts[method if method and z_obj is not None else 'NONE'] += 1
                self._log_method_stats()

                if z_obj is not None:
                    tag        = 'S' if method == 'STEREO_BBOX' else 'M'
                    disp_str   = f" d={disp:.1f}px" if disp is not None else ""
                    full_label = f"{z_obj:.2f}m [{tag}]{disp_str} {depth_conf:.0f}%"
                else:
                    full_label = "N/A"
                    depth_conf = 0.0

                # Affichage de la detection sur la frame gauche
                key_id       = cone_l['key_id']
                border_color = self.COLORS.get(key_id, (0, 255, 0))
                x1, y1, x2, y2 = cone_l['x1'], cone_l['y1'], cone_l['x2'], cone_l['y2']

                cv2.rectangle(display_frame, (x1, y1), (x2, y2), border_color, 2)
                (w_text, _), _ = cv2.getTextSize(
                    full_label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1
                )
                cv2.rectangle(
                    display_frame, (x1, y1 - 18), (x1 + w_text, y1), border_color, -1
                )
                cv2.putText(
                    display_frame, full_label, (x1, y1 - 4),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1
                )

                # Construction du message Detection2D
                w  = float(cone_l['w_box'])
                h  = float(cone_l['h_box'])
                cx = float(cone_l['cx'])
                cy = float(cone_l['cy'])

                detection        = Detection2D()
                detection.header = msg_left.header

                # Compatibilite Galactic / Iron
                try:
                    detection.bbox.center.x     = cx
                    detection.bbox.center.y     = cy
                    detection.bbox.center.theta = 0.0
                except AttributeError:
                    try:
                        detection.bbox.center.position.x = cx
                        detection.bbox.center.position.y = cy
                    except AttributeError:
                        pass

                detection.bbox.size_x = w
                detection.bbox.size_y = h

                hyp                     = ObjectHypothesisWithPose()
                hyp.hypothesis.class_id = str(key_id)

                # Convention de signe : positif = stereo (fiable), negatif = mono
                if method == 'STEREO_BBOX':
                    hyp.hypothesis.score = depth_conf / 100.0
                elif method == 'MONO':
                    hyp.hypothesis.score = -(depth_conf / 100.0)
                else:
                    hyp.hypothesis.score = 0.0

                # Stocke la position 3D dans le repère physique fsds/cam1
                # (X=avant, Y=gauche, Z=haut) — convention identique à sensor_fusion.py
                # Repère optique : z_opt=profondeur, x_opt=droite, y_opt=bas
                # Conversion optique → physique :
                #   x_phys =  z_opt   (profondeur → avant)
                #   y_phys = -x_opt   (gauche = -droite)
                #   z_phys = -y_opt   (haut = -bas)
                if z_obj is not None and z_obj > 0:
                    img_h, img_w = img_left.shape[:2]
                    fx    = self.focal_length
                    x_opt = (cone_l['cx'] - img_w / 2.0) * z_obj / fx
                    y_opt = (cone_l['cy'] - img_h / 2.0) * z_obj / fx
                    z_opt = z_obj
                    hyp.pose.pose.position.x =  z_opt   # avant
                    hyp.pose.pose.position.y = -x_opt   # gauche
                    hyp.pose.pose.position.z = -y_opt   # haut

                detection.results.append(hyp)
                detections_msg.detections.append(detection)

            self.publisher.publish(detections_msg)

            with self._display_lock:
                self._display_frame = display_frame

        except Exception as e:
            self.get_logger().error(f"Erreur Callback : {e}")


# ----------------------------------------------------------------------
# Boucle d'affichage dans un thread dedie
# Separation necessaire pour ne pas bloquer le spin ROS avec cv2.imshow
# ----------------------------------------------------------------------

def display_loop(node):
    import time
    while rclpy.ok():
        with node._display_lock:
            frame = node._display_frame
        if frame is not None:
            cv2.imshow("STEREO BBOX MATCHING", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        else:
            time.sleep(0.033)
    cv2.destroyAllWindows()


def main(args=None):
    rclpy.init(args=args)
    node = YoloStereoNode()

    # Thread daemon : termine automatiquement a la fin du process principal
    display_thread = threading.Thread(target=display_loop, args=(node,), daemon=True)
    display_thread.start()

    # MultiThreadedExecutor : execution parallele de camera_info_cb et sync_callback
    from rclpy.executors import MultiThreadedExecutor
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
