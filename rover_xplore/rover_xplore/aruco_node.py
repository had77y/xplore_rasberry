# ══════════════════════════════════════════════════════════════════════════════
# aruco_node.py — Raspberry Pi  (LifecycleNode)
#
# RÔLE : détecter les marqueurs ArUco dans le flux vidéo de la caméra
#         et publier leur position pixel pour autonomous_node.
#
# CYCLE DE VIE (géré par mode_manager_node) :
#   on_configure  → initialise le détecteur ArUco, crée le publisher
#   on_activate   → s'abonne à /camera/image_raw
#                   (camera_node voit un abonné → active la publication RAW)
#   on_deactivate → détruit la subscription, publie une détection vide
#                   (camera_node voit 0 abonnés → arrête la publication RAW)
#   on_cleanup    → libère le détecteur et le publisher
#
# TOPIC PUBLIÉ :
#   /aruco_detected  std_msgs/Float32MultiArray  [found, id, cx, cy, area]
#     found = 1.0 si détecté / 0.0 sinon
#     id    = identifiant du marker (-1.0 si rien)
#     cx,cy = centre du marker en pixels (origine haut-gauche)
#     area  = aire en pixels² (plus grand = plus proche)
#   Valeur quand rien détecté : [0.0, -1.0, 0.0, 0.0, 0.0]
#
# DICTIONNAIRE ARUCO (paramètre ROS2) :
#   ros2 run rover_xplore aruco_node --ros-args -p dictionary:=DICT_6X6_250
# ══════════════════════════════════════════════════════════════════════════════

import cv2
import numpy as np
import rclpy
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray

VIDEO_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)

DEFAULT_DICT = 'DICT_4X4_50'


class ArucoNode(LifecycleNode):

    def __init__(self):
        super().__init__('aruco_node')

        self._pub         = None
        self._sub_image   = None
        self._detect_fn   = None
        self._marker_visible = False

        # Déclaration du paramètre ici pour qu'il soit disponible dès le démarrage
        self.declare_parameter('dictionary', DEFAULT_DICT)

    # ── Lifecycle callbacks ───────────────────────────────────────────────────

    def on_configure(self, state):
        dict_name = self.get_parameter('dictionary').get_parameter_value().string_value
        dict_id   = getattr(cv2.aruco, dict_name, None)

        if dict_id is None:
            self.get_logger().error(f'Dictionnaire ArUco inconnu : {dict_name}')
            return TransitionCallbackReturn.FAILURE

        dictionary = cv2.aruco.getPredefinedDictionary(dict_id)

        if hasattr(cv2.aruco, 'ArucoDetector'):
            detector = cv2.aruco.ArucoDetector(dictionary, cv2.aruco.DetectorParameters())
            self._detect_fn = lambda gray: detector.detectMarkers(gray)
        else:
            params = cv2.aruco.DetectorParameters_create()
            self._detect_fn = lambda gray: cv2.aruco.detectMarkers(
                gray, dictionary, parameters=params
            )

        self._pub = self.create_publisher(Float32MultiArray, '/aruco_detected', 10)
        self.get_logger().info(f'aruco_node configuré — dict={dict_name}')
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state):
        self._marker_visible = False
        self._sub_image = self.create_subscription(
            Image, '/camera/image_raw', self._on_image, VIDEO_QOS
        )
        self.get_logger().info('aruco_node actif — détection ArUco activée')
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state):
        if self._sub_image is not None:
            self.destroy_subscription(self._sub_image)
            self._sub_image = None

        # Signale à autonomous_node qu'il n'y a plus de marker visible
        self._publish_empty()
        self._marker_visible = False
        self.get_logger().info('aruco_node inactif — détection désactivée')
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state):
        if self._pub is not None:
            self.destroy_publisher(self._pub)
            self._pub = None
        self._detect_fn = None
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state):
        self._publish_empty()
        return TransitionCallbackReturn.SUCCESS

    # ── Traitement de chaque frame ────────────────────────────────────────────

    def _on_image(self, msg: Image):
        if msg.encoding != 'bgr8':
            self.get_logger().warn(f'Encodage non géré : {msg.encoding} (attendu bgr8)')
            return

        frame = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
        gray  = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        corners, ids, _ = self._detect_fn(gray)

        out = Float32MultiArray()

        if ids is None or len(ids) == 0:
            if self._marker_visible:
                self.get_logger().info('ArUco perdu')
                self._marker_visible = False
            out.data = [0.0, -1.0, 0.0, 0.0, 0.0]
        else:
            # Garder le marker le plus proche (plus grande aire)
            areas = [cv2.contourArea(c[0]) for c in corners]
            idx   = int(np.argmax(areas))
            c     = corners[idx][0]
            cx    = float(np.mean(c[:, 0]))
            cy    = float(np.mean(c[:, 1]))
            marker_id = int(ids[idx][0])

            if not self._marker_visible:
                self.get_logger().info(
                    f'ArUco détecté — ID={marker_id} centre=({cx:.0f},{cy:.0f})'
                )
                self._marker_visible = True

            out.data = [1.0, float(marker_id), cx, cy, float(areas[idx])]

        self._pub.publish(out)

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _publish_empty(self):
        if self._pub is None:
            return
        out = Float32MultiArray()
        out.data = [0.0, -1.0, 0.0, 0.0, 0.0]
        self._pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = ArucoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
