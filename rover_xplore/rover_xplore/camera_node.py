# ──────────────────────────────────────────────────────────────────────────────
# FLUX DE DONNÉES — DOUBLE PUBLICATION
#
#   QUI ENVOIE  : camera_node (ce fichier) — tourne EN NATIF sur le Raspberry Pi
#   CE QU'IL ENVOIE :
#     → /camera/image_raw         (sensor_msgs/Image, bgr8, 640×480 @ 30 FPS)
#       Pour aruco_node (RPi) : qualité maximale, pas de pertes JPEG.
#       Reste LOCAL au Pi (DDS shared-memory ou loopback) → pas sur le WiFi.
#     → /camera/image_compressed  (sensor_msgs/CompressedImage, jpeg q80)
#       Pour video_viewer_node (PC) : ~1-2 MB/s sur le WiFi.
#
#   QUI REÇOIT  :
#     → aruco_node (RPi)        — sub /camera/image_raw
#     → video_viewer_node (PC)  — sub /camera/image_compressed
#
#   REMARQUE : ce node tourne HORS Docker (libcamera ne fonctionne pas dans Docker).
#              Les autres nodes tournent dans Docker avec --net=host → ils voient
#              ces topics automatiquement via ROS2 DDS.
#              QoS BEST_EFFORT sur les deux topics : les frames perdues sont
#              ignorées (pas de retransmission) → évite l'accumulation de lag.
# ──────────────────────────────────────────────────────────────────────────────

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import CompressedImage, Image
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

        # Double publication :
        #   - /camera/image_raw        → aruco_node (local Pi, qualité max)
        #   - /camera/image_compressed → video_viewer_node (PC via WiFi, JPEG)
        self.pub_raw = self.create_publisher(Image, '/camera/image_raw', VIDEO_QOS)
        self.pub_jpeg = self.create_publisher(CompressedImage, '/camera/image_compressed', VIDEO_QOS)

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
            # picamera2: "RGB888" => numpy en BGR (ordre attendu par OpenCV)
            main={"size": (FRAME_W, FRAME_H), "format": "RGB888"},
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

        stamp = self.get_clock().now().to_msg()

        # ── Publication RAW (bgr8) — pour aruco_node sur le Pi ──
        raw = Image()
        raw.header.stamp = stamp
        raw.header.frame_id = 'camera'
        raw.height = FRAME_H
        raw.width = FRAME_W
        raw.encoding = 'bgr8'
        raw.is_bigendian = 0
        raw.step = FRAME_W * 3
        raw.data = frame.tobytes()
        self.pub_raw.publish(raw)

        # ── Publication JPEG — pour video_viewer_node sur le PC ──
        ret, buf = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
        if not ret:
            self.get_logger().warn('Échec encodage JPEG — skip')
            return

        jpeg = CompressedImage()
        jpeg.header.stamp = stamp
        jpeg.header.frame_id = 'camera'
        jpeg.format = 'jpeg'
        jpeg.data = buf.tobytes()
        self.pub_jpeg.publish(jpeg)

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
