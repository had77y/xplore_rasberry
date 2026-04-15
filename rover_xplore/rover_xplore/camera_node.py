import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
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


class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')
        self.bridge = CvBridge()
        self.pub = self.create_publisher(Image, '/camera/image_raw', 10)

        if LIBCAMERA_AVAILABLE:
            self._init_libcamera()
        else:
            self._init_v4l2()

        self.timer = self.create_timer(1.0 / TARGET_FPS, self.capture_and_publish)
        backend = 'libcamera' if LIBCAMERA_AVAILABLE else 'V4L2'
        self.get_logger().info(
            f'camera_node démarré — {backend} {FRAME_W}x{FRAME_H} @ {TARGET_FPS} FPS'
        )

    # ------------------------------------------------------------------
    # Init backends
    # ------------------------------------------------------------------

    def _init_libcamera(self):
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
        self.cap = None
        for idx in range(32):
            cap = cv2.VideoCapture(idx)
            if cap.isOpened():
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_W)
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_H)
                cap.set(cv2.CAP_PROP_FPS, TARGET_FPS)
                self.cap = cap
                self.get_logger().info(f'V4L2: utilise /dev/video{idx}')
                break
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
        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera'
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
