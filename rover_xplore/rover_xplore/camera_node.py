import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from picamera2 import Picamera2

class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')

        self.image_pub = self.create_publisher(Image, '/camera/image_raw', 10)
        self.bridge = CvBridge()

        self.camera = Picamera2()
        config = self.camera.create_still_configuration(
                  main={"size": (640, 480), "format": "RGB888"}
        )
        self.camera.configure(config)
        self.camera.start()

        self.camera.capture_array()  # capture une image pour initialiser la caméra, sinon la première image est noire

        self.timer = self.create_timer(0.066, self.capture_frame)  # appelle capture_frame toutes les 66 ms, soit ~15 FPS

        self.get_logger().info('camera_node started')

    def capture_frame(self):
        frame = self.camera.capture_array()              # capture l'image (numpy array RGB888)
        msg = self.bridge.cv2_to_imgmsg(frame, 'rgb8')  # convertit en message ROS2
        msg.header.stamp = self.get_clock().now().to_msg()
        self.image_pub.publish(msg)                      # publie sur /camera/image_raw


def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
