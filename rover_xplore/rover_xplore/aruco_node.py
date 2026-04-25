# ──────────────────────────────────────────────────────────────────────────────
# FLUX DE DONNÉES
#
#   QUI ENVOIE  : aruco_node (ce fichier) — tourne sur le Raspberry Pi
#   CE QU'IL ENVOIE : la détection ArUco du marker le plus proche (plus grande aire)
#   TOPIC       : /aruco_detected  (std_msgs/Float32MultiArray)
#                 data = [found(0/1), id, center_x, center_y, area_px²]
#                 Si rien détecté : [0.0, -1.0, 0.0, 0.0, 0.0]
#
#   QUI ÉCOUTE :
#     /rover/mode        — active/désactive la subscription image
#     /camera/image_raw  — subscrit SEULEMENT en mode 'autonomous'
#                          → camera_node voit 0 subscribers hors autonomous
#                          → camera_node skip le raw automatiquement → pas de latence
#
#   QUI REÇOIT  :
#     → autonomous_node (RPi, package rover_commands)
#
#   PARAMÈTRE  : `dictionary` (default DICT_4X4_50)
#                ros2 run rover_xplore aruco_node --ros-args -p dictionary:=DICT_6X6_250
# ──────────────────────────────────────────────────────────────────────────────

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray, String
import numpy as np
import cv2

VIDEO_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)

DEFAULT_DICT = 'DICT_4X4_50'


class ArucoNode(Node):
    def __init__(self):
        super().__init__('aruco_node')

        self.declare_parameter('dictionary', DEFAULT_DICT)
        dict_name = self.get_parameter('dictionary').get_parameter_value().string_value

        dict_id = getattr(cv2.aruco, dict_name, None)
        if dict_id is None:
            self.get_logger().error(f'Dictionnaire ArUco inconnu: {dict_name}')
            raise ValueError(f'Unknown ArUco dictionary: {dict_name}')

        dictionary = cv2.aruco.getPredefinedDictionary(dict_id)
        # Supporte OpenCV >= 4.7 (ArucoDetector) et les versions antérieures
        if hasattr(cv2.aruco, 'ArucoDetector'):
            parameters = cv2.aruco.DetectorParameters()
            self._detector = cv2.aruco.ArucoDetector(dictionary, parameters)
            self._detect_fn = lambda gray: self._detector.detectMarkers(gray)
        else:
            parameters = cv2.aruco.DetectorParameters_create()
            self._detect_fn = lambda gray: cv2.aruco.detectMarkers(
                gray, dictionary, parameters=parameters
            )

        # Subscription image — créée dynamiquement selon le mode
        self._sub_image = None
        self._marker_visible = False  # pour log uniquement sur transition

        self.create_subscription(String, '/rover/mode', self._on_mode, 10)
        self.pub = self.create_publisher(Float32MultiArray, '/aruco_detected', 10)

        self.get_logger().info(f'aruco_node démarré — dict={dict_name} — en attente mode autonomous')

    def _on_mode(self, msg: String):
        if msg.data == 'autonomous' and self._sub_image is None:
            self._sub_image = self.create_subscription(
                Image, '/camera/image_raw', self._on_image, VIDEO_QOS
            )
            self.get_logger().info('Mode autonomous — détection ArUco activée')
        elif msg.data != 'autonomous' and self._sub_image is not None:
            self.destroy_subscription(self._sub_image)
            self._sub_image = None
            # Publier "rien détecté" pour reset autonomous_node
            out = Float32MultiArray()
            out.data = [0.0, -1.0, 0.0, 0.0, 0.0]
            self.pub.publish(out)
            self.get_logger().info(f'Mode {msg.data} — détection ArUco désactivée')

    def _on_image(self, msg: Image):
        if msg.encoding != 'bgr8':
            self.get_logger().warn(f'Encodage non géré: {msg.encoding} (attendu bgr8)')
            return

        frame = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self._detect_fn(gray)

        out = Float32MultiArray()
        if ids is None or len(ids) == 0:
            if self._marker_visible:
                self.get_logger().info('ArUco perdu')
                self._marker_visible = False
            out.data = [0.0, -1.0, 0.0, 0.0, 0.0]
        else:
            # Marker le plus proche = celui avec la plus grande aire
            areas = [cv2.contourArea(c[0]) for c in corners]
            idx = int(np.argmax(areas))
            c = corners[idx][0]
            cx = float(np.mean(c[:, 0]))
            cy = float(np.mean(c[:, 1]))
            marker_id = int(ids[idx][0])
            if not self._marker_visible:
                self.get_logger().info(f'ArUco détecté ! ID={marker_id} centre=({cx:.0f},{cy:.0f})')
                self._marker_visible = True
            out.data = [1.0, float(marker_id), cx, cy, float(areas[idx])]

        self.pub.publish(out)


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
