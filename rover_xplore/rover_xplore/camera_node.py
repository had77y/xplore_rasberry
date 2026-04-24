# ──────────────────────────────────────────────────────────────────────────────
# FLUX DE DONNÉES
#
#   QUI ENVOIE  : camera_node (ce fichier) — tourne EN NATIF sur le Raspberry Pi
#   CE QU'IL ENVOIE : une image JPEG compressée 640×480 @ 30 FPS (~1-2 MB/s)
#   TOPIC       : /camera/image_compressed  (sensor_msgs/CompressedImage, jpeg)
#
#   QUI REÇOIT  :
#     → video_viewer_node (PC)   — affiche le flux FPV en temps réel
#     → aruco_node (RPi)         — détecte le tag ArUco (décode JPEG localement)
#
#   REMARQUE : ce node tourne HORS Docker (libcamera ne fonctionne pas dans Docker).
#              Les autres nodes tournent dans Docker avec --net=host → ils voient
#              ce topic automatiquement via ROS2 DDS.
#              QoS BEST_EFFORT : les frames perdues sont ignorées (pas de retransmission)
#              → évite l'accumulation de lag sur WiFi.
# ──────────────────────────────────────────────────────────────────────────────

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import CompressedImage
import cv2

try:
    from picamera2 import Picamera2
    LIBCAMERA_AVAILABLE = True
except ImportError:
    LIBCAMERA_AVAILABLE = False

FRAME_W = 640
FRAME_H = 480
TARGET_FPS = 30
WARMUP_FRAMES = TARGET_FPS  # ~1s de drain AEC/AWB
JPEG_QUALITY = 80

VIDEO_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')

        # Publication des frames — reçues par video_viewer_node (PC) et aruco_node (RPi)
        self.pub = self.create_publisher(CompressedImage, '/camera/image_compressed', VIDEO_QOS)

        libcamera_cameras = Picamera2.global_camera_info() if LIBCAMERA_AVAILABLE else []
        if libcamera_cameras:
            self._init_libcamera()
        else:
            if LIBCAMERA_AVAILABLE:
                self.get_logger().warn(
                    'libcamera disponible mais aucune caméra CSI détectée '
                    '(vérifier câble CSI / dtoverlay) — fallback V4L2'
                )
            self._init_v4l2()

        self.timer = self.create_timer(1.0 / TARGET_FPS, self.capture_and_publish)
        backend = 'libcamera' if libcamera_cameras else 'V4L2'
        self.get_logger().info(
            f'camera_node démarré — {backend} {FRAME_W}x{FRAME_H} @ {TARGET_FPS} FPS '
            f'(JPEG q{JPEG_QUALITY}, BEST_EFFORT)'
        )

    # ------------------------------------------------------------------
    # Init backends
    # ------------------------------------------------------------------

    def _init_libcamera(self):
        # Pi Camera CSI — backend natif libcamera via picamera2
        self.cam = Picamera2()
        frame_duration_us = int(1_000_000 / TARGET_FPS)
        cfg = self.cam.create_video_configuration(
            main={"size": (FRAME_W, FRAME_H), "format": "BGR888"},
            controls={"FrameDurationLimits": (frame_duration_us, frame_duration_us)},
        )
        self.cam.configure(cfg)
        self.cam.start()
        # Drain des premières frames pour laisser AEC/AWB converger
        for _ in range(WARMUP_FRAMES):
            self.cam.capture_array("main")
        self.capture_fn = lambda: self.cam.capture_array("main")

    def _init_v4l2(self):
        # Fallback USB / VM — scan automatique des devices V4L2
        self.cap = None
        for idx in range(32):
            path = f'/dev/video{idx}'
            cap = cv2.VideoCapture(path, cv2.CAP_V4L)
            if cap.isOpened():
                cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc('M', 'J', 'P', 'G'))
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_W)
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_H)
                cap.set(cv2.CAP_PROP_FPS, TARGET_FPS)
                ret, _ = cap.read()
                if ret:
                    self.cap = cap
                    self.get_logger().info(f'V4L2: utilise {path} (MJPG)')
                    break
                cap.release()
            else:
                cap.release()
        if self.cap is None:
            self.get_logger().error('Aucune caméra V4L2 disponible !')
            raise RuntimeError('Camera introuvable')
        self.capture_fn = self._capture_v4l2

    def _capture_v4l2(self):
        ret, frame = self.cap.read()
        return frame if ret else None

    # ------------------------------------------------------------------
    # Boucle principale
    # ------------------------------------------------------------------

    def capture_and_publish(self):
        frame = self.capture_fn()
        if frame is None:
            self.get_logger().warn('Frame vide — skip')
            return

        ret, buf = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
        if not ret:
            self.get_logger().warn('Échec encodage JPEG — skip')
            return

        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera'
        msg.format = 'jpeg'
        msg.data = buf.tobytes()
        self.pub.publish(msg)

    # ------------------------------------------------------------------
    # Cleanup
    # ------------------------------------------------------------------

    def destroy_node(self):
        if LIBCAMERA_AVAILABLE and hasattr(self, 'cam'):
            self.cam.stop()
        elif hasattr(self, 'cap'):
            self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
