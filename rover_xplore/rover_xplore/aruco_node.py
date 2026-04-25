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
#     /camera/image_raw  (sensor_msgs/Image, bgr8) — publié par camera_node
#                                                   (RAW = qualité max, intra-Pi)
#
#   QUI REÇOIT  :
#     → autonomous_node (RPi, package rover_commands) — utilise pour SEARCH /
#                       CONFIRM_ARUCO / approche visuelle
#
#   PARAMÈTRE  : `dictionary` (default DICT_4X4_50)
#                ros2 run rover_xplore aruco_node --ros-args -p dictionary:=DICT_6X6_250
# ──────────────────────────────────────────────────────────────────────────────

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
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
            self._detect_fn = lambda gray: cv2.aruco.detectMarkers(gray, dictionary, parameters=parameters)

        self.sub = self.create_subscription(
            Image, '/camera/image_raw', self._on_image, VIDEO_QOS
        )
        self.pub = self.create_publisher(Float32MultiArray, '/aruco_detected', 10)

        self.get_logger().info(f'aruco_node démarré — dict={dict_name}')

    def _on_image(self, msg: Image):
        if msg.encoding != 'bgr8':
            self.get_logger().warn(f'Encodage non géré: {msg.encoding} (attendu bgr8)')
            return

        frame = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self._detect_fn(gray)

        out = Float32MultiArray()
        if ids is None or len(ids) == 0:
            out.data = [0.0, -1.0, 0.0, 0.0, 0.0]
        else:
            # Marker le plus proche = celui avec la plus grande aire
            areas = [cv2.contourArea(c[0]) for c in corners]
            idx = int(np.argmax(areas))
            c = corners[idx][0]
            cx = float(np.mean(c[:, 0]))
            cy = float(np.mean(c[:, 1]))
            out.data = [1.0, float(ids[idx][0]), cx, cy, float(areas[idx])]

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
