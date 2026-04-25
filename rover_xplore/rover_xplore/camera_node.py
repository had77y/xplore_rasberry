# ──────────────────────────────────────────────────────────────────────────────
# FLUX DE DONNÉES — DOUBLE PUBLICATION, GÉRÉ PAR /rover/mode
#
#   QUI ENVOIE  : camera_node (ce fichier) — tourne EN NATIF sur le Raspberry Pi
#   CE QU'IL ENVOIE :
#     → /camera/image_compressed  (sensor_msgs/CompressedImage, jpeg q80)
#       Actif en modes : race, autonomous
#       Pour video_viewer_node (PC) : ~1-2 MB/s sur le WiFi.
#     → /camera/image_raw         (sensor_msgs/Image, bgr8, 640×480 @ 30 FPS)
#       Actif uniquement si aruco_node est subscrit (mode autonomous)
#       Pour aruco_node (RPi) : qualité maximale, reste local au Pi.
#
#   MODE idle / arm → le node tourne mais ne publie rien (0% CPU caméra).
#
#   REMARQUE : ce node tourne HORS Docker (libcamera ne fonctionne pas dans Docker).
#              Les autres nodes tournent dans Docker avec --net=host → ils voient
#              ces topics automatiquement via ROS2 DDS.
# ──────────────────────────────────────────────────────────────────────────────

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import String
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

ACTIVE_MODES = {'race', 'autonomous'}

VIDEO_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')

        self._active = False  # publie seulement si mode race ou autonomous

        self.pub_raw = self.create_publisher(Image, '/camera/image_raw', VIDEO_QOS)
        self.pub_jpeg = self.create_publisher(CompressedImage, '/camera/image_compressed', VIDEO_QOS)
        self.create_subscription(String, '/rover/mode', self._on_mode, 10)

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
            f'(JPEG q{JPEG_QUALITY}) — en attente du mode...'
        )

    # ------------------------------------------------------------------
    # Mode
    # ------------------------------------------------------------------

    def _on_mode(self, msg: String):
        was_active = self._active
        self._active = msg.data in ACTIVE_MODES
        if self._active and not was_active:
            self.get_logger().info(f'Mode {msg.data} — caméra activée')
        elif not self._active and was_active:
            self.get_logger().info(f'Mode {msg.data} — caméra désactivée')

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
        if not self._active:
            return

        frame = self.capture_fn()
        if frame is None:
            self.get_logger().warn('Frame vide — skip')
            return

        stamp = self.get_clock().now().to_msg()

        # ── JPEG en premier — minimise la latence du viewer PC ──
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

        # ── RAW (bgr8) — seulement si aruco_node est actif (mode autonomous) ──
        if self.pub_raw.get_subscription_count() > 0:
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
