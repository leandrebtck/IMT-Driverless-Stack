#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
import message_filters
from ultralytics import YOLO
import os
import torch
import threading

from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose

# Hauteur reelle d'un cone FSG selon le reglement technique Formula Student
FSG_CONE_REAL_HEIGHT_M = 0.325

# Ecartement entre cam1 (Y=-0.32) et cam2 (Y=+0.32) defini dans settings.json
BASELINE_M = 0.64

# Focale par defaut en pixels avant reception du topic camera_info
# Calcul : f = W / (2 * tan(FOV/2)) = 416 / (2 * tan(45deg)) = 208px
# Remplacee dynamiquement par la valeur du camera_info au demarrage
FOCAL_PX = 208.0

# Age maximum acceptable d'une image depth par rapport a la frame courante.
# Au-dela de cette valeur, la mesure depth est consideree perimee et ignoree.
DEPTH_MAX_AGE_S = 0.5

# Nombre minimum de pixels valides requis dans un ROI pour accepter une mesure.
# Une valeur de 1 permet de traiter les cones lointains dont la bbox est petite.
MIN_VALID_PX = 1

# Fraction de la hauteur de la bbox conservee en partant du bas.
# La partie haute d'une bbox contient souvent du fond ou du ciel,
# ce qui biaiserait la mesure de profondeur vers des valeurs plus grandes.
BOTTOM_ROI_RATIO = 0.35

# Valeur minimale de disparite SGBM acceptee comme valide.
# Les pixels a disparite quasi-nulle correspondent a des zones sans correspondance
# et produiraient des profondeurs aberrantes via Z = f*B/d.
DISP_MIN_THRESHOLD = 0.5


class YoloStereoNode(Node):
    """
    Node ROS 2 de perception visuelle pour Formula Student Driverless.

    Pipeline :
      1. Detection des cones par YOLO sur l'image gauche.
      2. Estimation de profondeur par cascade de trois methodes :
         a. Depth camera (prioritaire, ground-truth simulateur).
         b. Stereoscopie SGBM (calcul lazy, une seule fois par frame).
         c. Monoculaire par taille apparente (dernier recours).
      3. Correction de classe par analyse HSV pour les faux positifs orange
         dus a un eclairage chaud.
      4. Publication des detections sur /perception/stereo_detections.
    """

    def __init__(self):
        super().__init__('yolo_stereo_node')
        self.bridge = CvBridge()

        self.baseline     = BASELINE_M
        self.focal_length = FOCAL_PX
        self.fx           = None  # rempli une seule fois a la reception du camera_info

        self.latest_depth       = None
        self.latest_depth_stamp = None

        self._display_frame = None
        self._display_lock  = threading.Lock()

        # Compteurs de methodes pour monitoring en production
        self._method_counts = {'DEPTH': 0, 'STEREO': 0, 'MONO': 0, 'NONE': 0}
        self._log_interval  = 100
        self._total_dets    = 0

        # Chargement du modele YOLO depuis le dossier weights/ relatif au script
        script_dir   = os.path.dirname(os.path.abspath(__file__))
        weights_path = os.path.join(script_dir, 'weights', 'best_FINAL.pt')

        self.get_logger().info(f"Dossier du script : {script_dir}")

        if not os.path.exists(weights_path):
            # Fallback : cherche le .pt a la racine du dossier script
            if os.path.exists(os.path.join(script_dir, 'best_FINAL.pt')):
                weights_path = os.path.join(script_dir, 'best_FINAL.pt')
            else:
                raise FileNotFoundError("Impossible de trouver best_FINAL.pt")

        self.model = YOLO(weights_path)
        device = 'cuda' if torch.cuda.is_available() else 'cpu'
        self.model.to(device)
        self.get_logger().info(f"YOLO lance sur : {device}")

        # CLAHE applique sur le canal L de l'espace LAB avant SGBM.
        # Normalise le contraste localement pour ameliorer la qualite
        # des correspondances stereo sous eclairages non uniformes.
        self.clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))

        # Algorithme Semi-Global Block Matching pour la carte de disparite.
        # blockSize=3 : petite fenetre pour plus de detail sur les petits objets.
        # P1/P2 : penalites de regularite ; P2 >> P1 pour des surfaces lisses.
        self.left_matcher = cv2.StereoSGBM_create(
            minDisparity=0,
            numDisparities=128,
            blockSize=3,
            P1=8  * 3 * 3 ** 2,
            P2=32 * 3 * 3 ** 2,
            disp12MaxDiff=1,
            uniquenessRatio=5,
            speckleWindowSize=80,
            speckleRange=16,
            mode=cv2.STEREO_SGBM_MODE_SGBM
        )

        # Couleurs et noms d'affichage par classe FSG
        # 0 = jaune, 1 = bleu, 2 = orange/grand
        self.COLORS       = {0: (0, 255, 255), 1: (255, 0, 0), 2: (0, 140, 255)}
        self.CUSTOM_NAMES = {0: "JAUNE", 1: "BLEU", 2: "ORANGE"}

        # Souscription simple : camera_info et depth n'ont pas besoin
        # d'etre synchronises avec les cameras couleur
        self.sub_cam_info = self.create_subscription(
            CameraInfo, '/fsds/cam1/camera_info', self.camera_info_cb, 10
        )
        self.sub_depth = self.create_subscription(
            Image, '/fsds/cam_depth/image_depth',
            self.depth_cb, qos_profile_sensor_data
        )

        # Synchronisation temporelle stricte entre cam1 et cam2.
        # Les deux images doivent etre capturees au meme instant
        # pour que la disparite SGBM soit geometriquement correcte.
        # Tolerance : 50ms entre les deux timestamps.
        self.sub_left = message_filters.Subscriber(
            self, Image, '/fsds/cam1/image_color',
            qos_profile=qos_profile_sensor_data
        )
        self.sub_right = message_filters.Subscriber(
            self, Image, '/fsds/cam2/image_color',
            qos_profile=qos_profile_sensor_data
        )
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.sub_left, self.sub_right], 10, 0.05
        )
        self.ts.registerCallback(self.sync_callback)

        self.publisher = self.create_publisher(
            Detection2DArray, '/perception/stereo_detections', 10
        )

        self.get_logger().info("Perception Stereo + Depth lancee.")
        self.get_logger().info(
            f"Parametres : DEPTH_MAX_AGE={DEPTH_MAX_AGE_S}s | "
            f"MIN_VALID_PX={MIN_VALID_PX} | "
            f"BOTTOM_ROI_RATIO={BOTTOM_ROI_RATIO} | "
            f"DISP_MIN={DISP_MIN_THRESHOLD}"
        )

    # ------------------------------------------------------------------
    # Camera info
    # ------------------------------------------------------------------

    def camera_info_cb(self, msg):
        """Recupere la focale reelle depuis la matrice intrinseque K.
        K est un tableau plat de 9 valeurs : K[0] = fx."""
        if self.fx is None:
            self.fx           = msg.k[0]
            self.focal_length = self.fx
            self.get_logger().info(f"Focale recue via camera_info : {self.focal_length:.2f} px")

    # ------------------------------------------------------------------
    # Depth sensor
    # ------------------------------------------------------------------

    def depth_cb(self, msg):
        """Stocke la derniere image depth recue avec son timestamp."""
        try:
            self.latest_depth       = self.bridge.imgmsg_to_cv2(msg, "32FC1")
            self.latest_depth_stamp = msg.header.stamp
        except Exception as e:
            self.get_logger().warn(f"Erreur conversion depth : {e}")

    def _depth_is_fresh(self, ref_stamp):
        """Retourne True si l'image depth est suffisamment recente
        par rapport au timestamp de la frame courante."""
        if self.latest_depth is None or self.latest_depth_stamp is None:
            return False
        t_depth = self.latest_depth_stamp.sec + self.latest_depth_stamp.nanosec * 1e-9
        t_ref   = ref_stamp.sec + ref_stamp.nanosec * 1e-9
        return abs(t_ref - t_depth) < DEPTH_MAX_AGE_S

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    @staticmethod
    def _bottom_roi(y1, y2, ratio=BOTTOM_ROI_RATIO):
        """Retourne (y1_crop, y2) correspondant aux 'ratio' inferieurs de la bbox.
        Isole la base du cone et exclut le fond visible en haut de la detection."""
        h = y2 - y1
        return int(y1 + h * (1.0 - ratio)), y2

    def _log_method_stats(self):
        """Log la distribution des methodes d'estimation toutes les
        _log_interval detections pour le monitoring en production."""
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
    # Estimation de profondeur — cascade de trois methodes
    # ------------------------------------------------------------------

    def estimate_depth_camera(self, ref_stamp, x1, y1, x2, y2):
        """Methode 1 : profondeur directe depuis le capteur depth simulateur.
        Lit la partie inferieure de la bbox pour eviter les pixels de fond.
        Utilise le percentile 15 pour cibler les pixels les plus proches (le cone),
        en ignorant les pixels residuels du fond plus lointain."""
        if not self._depth_is_fresh(ref_stamp):
            return None, None

        img_depth        = self.latest_depth
        y1_crop, y2_crop = self._bottom_roi(y1, y2)

        roi   = img_depth[max(0, y1_crop):min(img_depth.shape[0], y2_crop),
                          max(0, x1):min(img_depth.shape[1], x2)]
        valid = roi[(roi > 0.0) & (roi < 50.0)]

        if len(valid) < MIN_VALID_PX:
            return None, None

        z = float(np.percentile(valid, 15))
        return (z, 'DEPTH') if z > 0.0 else (None, None)

    def estimate_depth_stereo(self, disparity_f32, x1, y1, x2, y2):
        """Methode 2 : profondeur par triangulation stereoscopique.
        Formule : Z = f * B / d  (focale * baseline / disparite)
        Utilise le percentile 85 sur la disparite, equivalent au percentile 15
        sur la profondeur : cible l'objet au premier plan.
        Rejette les mesures au-dela de 20m ou la stereo n'est plus fiable
        avec une baseline de 0.64m sur des objets peu textures."""
        y1_crop, y2_crop = self._bottom_roi(y1, y2)

        roi_disp   = disparity_f32[max(0, y1_crop):min(disparity_f32.shape[0], y2_crop),
                                   max(0, x1):min(disparity_f32.shape[1], x2)]
        valid_disp = roi_disp[roi_disp > DISP_MIN_THRESHOLD]

        if len(valid_disp) < MIN_VALID_PX:
            return None, None

        d_obj = np.percentile(valid_disp, 85)
        if d_obj < DISP_MIN_THRESHOLD:
            return None, None

        z = (self.focal_length * self.baseline) / d_obj
        return (z, 'STEREO') if z <= 20.0 else (None, None)

    def estimate_depth_monocular(self, bbox_height_px):
        """Methode 3 : profondeur par taille apparente du cone (dernier recours).
        Formule : Z = f * H_reel / H_px
        Hypothese : la hauteur reelle du cone est connue et constante (FSG = 0.325m).
        Precision limitee par la qualite de la bbox YOLO et la perspective."""
        if bbox_height_px < 5:
            return None, None
        return (self.focal_length * FSG_CONE_REAL_HEIGHT_M) / bbox_height_px, 'MONO'

    # ------------------------------------------------------------------
    # Validation de classe par couleur HSV
    # ------------------------------------------------------------------

    def validate_class_by_color(self, img_bgr, x1, y1, x2, y2, key_id):
        """Verifie la coherence entre la classe YOLO et la couleur mesuree en HSV.
        Corrige les faux positifs orange causes par un eclairage chaud (coucher de soleil)
        qui teinte les cones jaunes et bleus dans des teintes orangees.
        La correction est conservative : elle s'applique uniquement si
        la classe predite est orange et que la couleur dominante mesuree
        est clairement une autre classe (seuil de 8% des pixels)."""
        roi = img_bgr[max(0, y1):min(img_bgr.shape[0], y2),
                      max(0, x1):min(img_bgr.shape[1], x2)]
        if roi.size == 0:
            return key_id

        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        # Plages HSV des cones FSG en conditions d'eclairage standard
        # Format : (H_min, S_min, V_min), (H_max, S_max, V_max)
        # Saturation minimale de 80 pour exclure les zones grises / surexposees
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
    # Score de confiance par methode
    # ------------------------------------------------------------------

    def compute_confidence(self, method, z, valid_count=0, disp_mean=0.0):
        """Calcule un score de confiance 0-100% selon la methode utilisee.

        DEPTH  : confiance de base 75%, bonifiee par la proximite de l'objet.
        STEREO : combinaison de trois sous-scores ponderes :
                   - valid_score (35%) : densite de pixels valides dans le ROI
                   - disp_score  (40%) : disparite moyenne (plus grande = plus proche = plus fiable)
                   - z_score     (25%) : bonus de proximite
        MONO   : proportionnel a la taille de la bbox en pixels.
                 Une grande bbox = cone proche = estimation plus precise."""
        if method is None or z is None or z <= 0:
            return 0.0

        if method == 'DEPTH':
            z_score = max(0.0, (50.0 - z) / 50.0)
            conf    = (0.75 + 0.25 * z_score) * 100.0

        elif method == 'STEREO':
            valid_score = min(valid_count / 100.0, 1.0)
            disp_score  = min(disp_mean / 80.0, 1.0) if disp_mean > 0 else 0.0
            z_score     = max(0.0, (20.0 - z) / 20.0)
            conf        = (0.35 * valid_score + 0.40 * disp_score + 0.25 * z_score) * 100.0

        elif method == 'MONO':
            bbox_h_est = (self.focal_length * FSG_CONE_REAL_HEIGHT_M) / z
            conf       = min(bbox_h_est / 30.0, 1.0) * 100.0

        else:
            conf = 0.0

        return round(conf, 1)

    # ------------------------------------------------------------------
    # Callback principal — declenche a chaque paire d'images synchronisees
    # ------------------------------------------------------------------

    def sync_callback(self, msg_left, msg_right):
        try:
            img_left      = self.bridge.imgmsg_to_cv2(msg_left, "bgr8")
            display_frame = img_left.copy()

            with torch.no_grad():
                results = self.model(img_left, verbose=False, conf=0.5)

            # Initialisation lazy : SGBM n'est calcule que si depth echoue,
            # et seulement une fois par frame meme pour plusieurs cones.
            disparity_f32 = None
            img_right_cv  = None

            detections_msg        = Detection2DArray()
            detections_msg.header = msg_left.header

            if len(results) > 0:
                for box in results[0].boxes:
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    cls_id     = int(box.cls[0])
                    yolo_conf  = float(box.conf[0])
                    class_name = self.model.names[cls_id].lower()

                    # Mapping des noms de classes YOLO vers les identifiants FSG
                    # Les cones "large" correspondent aux cones oranges de grande taille
                    if "orange" in class_name or "large" in class_name:
                        key_id = 2
                    elif "blue" in class_name:
                        key_id = 1
                    elif "yellow" in class_name:
                        key_id = 0
                    else:
                        key_id = cls_id

                    # Validation de la classe par analyse HSV
                    key_id = self.validate_class_by_color(img_left, x1, y1, x2, y2, key_id)

                    w_box = x2 - x1
                    h_box = y2 - y1

                    # Cascade d'estimation de profondeur
                    z_obj, method = self.estimate_depth_camera(
                        msg_left.header.stamp, x1, y1, x2, y2
                    )

                    if z_obj is None:
                        # Calcul de la carte de disparite SGBM si pas encore fait pour cette frame.
                        # Preprocessing LAB + CLAHE sur le canal L uniquement
                        # pour normaliser le contraste sans alterer la chrominance.
                        if disparity_f32 is None:
                            img_right_cv  = self.bridge.imgmsg_to_cv2(msg_right, "bgr8")
                            lab_left      = cv2.cvtColor(img_left,     cv2.COLOR_BGR2LAB)
                            lab_right     = cv2.cvtColor(img_right_cv, cv2.COLOR_BGR2LAB)
                            lab_left[:, :, 0]  = self.clahe.apply(lab_left[:, :, 0])
                            lab_right[:, :, 0] = self.clahe.apply(lab_right[:, :, 0])
                            gray_left  = cv2.cvtColor(
                                cv2.cvtColor(lab_left,  cv2.COLOR_LAB2BGR), cv2.COLOR_BGR2GRAY
                            )
                            gray_right = cv2.cvtColor(
                                cv2.cvtColor(lab_right, cv2.COLOR_LAB2BGR), cv2.COLOR_BGR2GRAY
                            )
                            # Division par 16 : SGBM retourne des valeurs en Q11.4 (fixed-point)
                            disparity_f32 = (
                                self.left_matcher.compute(gray_left, gray_right)
                                .astype(np.float32) / 16.0
                            )

                        z_obj, method = self.estimate_depth_stereo(
                            disparity_f32, x1, y1, x2, y2
                        )

                    if z_obj is None:
                        z_obj, method = self.estimate_depth_monocular(h_box)

                    # Mise a jour des compteurs de monitoring
                    self._method_counts[method if method else 'NONE'] += 1
                    self._log_method_stats()

                    # Calcul des statistiques de disparite pour le score STEREO
                    disp_mean_val = 0.0
                    valid_count   = 0
                    if method == 'STEREO' and disparity_f32 is not None:
                        y1_crop, y2_crop = self._bottom_roi(y1, y2)
                        roi_disp_dbg  = disparity_f32[
                            max(0, y1_crop):min(disparity_f32.shape[0], y2_crop),
                            max(0, x1):min(disparity_f32.shape[1], x2)
                        ]
                        valid_dbg     = roi_disp_dbg[roi_disp_dbg > DISP_MIN_THRESHOLD]
                        disp_mean_val = float(roi_disp_dbg.mean()) if roi_disp_dbg.size > 0 else 0.0
                        valid_count   = len(valid_dbg)

                    depth_conf = self.compute_confidence(
                        method, z_obj,
                        valid_count=valid_count,
                        disp_mean=disp_mean_val
                    )

                    # Correction empirique de biais systematique par couleur.
                    # Biais observe en simulation sur SGBM et depth camera :
                    # les cones bleus sont surestimes d'environ 2m,
                    # les cones jaunes d'environ 1.5m.
                    # Non applique sur MONO dont le biais est de nature differente.
                    DEPTH_OFFSET = {0: -1.5, 1: -2.0, 2: 0.0}
                    if z_obj is not None and method != 'MONO':
                        z_obj = max(0.1, z_obj + DEPTH_OFFSET.get(key_id, 0.0))

                    if z_obj is not None:
                        tag        = {'DEPTH': 'D', 'STEREO': 'S', 'MONO': 'M'}.get(method, '?')
                        full_label = f"{z_obj:.2f}m [{tag}] {depth_conf:.0f}%"
                    else:
                        full_label = "N/A"
                        depth_conf = 0.0

                    # Affichage de la bbox et du label sur la frame de debug
                    border_color = self.COLORS.get(key_id, (0, 255, 0))
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
                    w  = float(w_box)
                    h  = float(h_box)
                    cx = float(x1 + w / 2.0)
                    cy = float(y1 + h / 2.0)

                    detection        = Detection2D()
                    detection.header = msg_left.header

                    # Compatibilite Galactic / Iron : le champ center a change de structure
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

                    # Convention de signe du score pour les modules aval :
                    # score positif  = mesure instrumentale (DEPTH ou STEREO)
                    # score negatif  = estimation monoculaire (moins fiable)
                    # score nul      = echec de toutes les methodes
                    if method in ('DEPTH', 'STEREO'):
                        hyp.hypothesis.score = depth_conf / 100.0
                    elif method == 'MONO':
                        hyp.hypothesis.score = -(depth_conf / 100.0)
                    else:
                        hyp.hypothesis.score = 0.0

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
            cv2.imshow("STEREO + DEPTH FUSION", frame)
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

    # MultiThreadedExecutor : permet l'execution parallele des callbacks
    # (depth_cb, camera_info_cb et sync_callback tournent sur des threads distincts)
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
