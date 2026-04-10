import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from picamera2 import Picamera2
import time


class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')

        self.image_pub = self.create_publisher(Image, '/camera/image_raw', 10)
        self.bridge = CvBridge()

        self.camera = Picamera2()

        # create_video_configuration pour le streaming continu (pas still qui donne des frames noires)
        config = self.camera.create_video_configuration(
            main={"size": (640, 480), "format": "RGB888"}
        )
        self.camera.configure(config)
        self.camera.start()

        # Laisser le temps au capteur de s'initialiser (AEC/AWB convergence)
        time.sleep(2.0)

        # Warmup : vider le buffer de démarrage
        for _ in range(5):
            self.camera.capture_array("main")

        self.get_logger().info('camera_node started — video config, warmup done')
        self.timer = self.create_timer(0.066, self.capture_frame)  # ~15 FPS

    def capture_frame(self):
        frame = self.camera.capture_array("main")  # numpy array RGB888
        if frame is None:
            self.get_logger().warn('Frame vide !')
            return
        msg = self.bridge.cv2_to_imgmsg(frame, 'rgb8')
        msg.header.stamp = self.get_clock().now().to_msg()
        self.image_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
