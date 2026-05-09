# ══════════════════════════════════════════════════════════════════════════════
# motor_controller_node.py — Raspberry Pi
#
# RÔLE : convertir les commandes de déplacement (Twist) en vitesses par roue
#         et les publier sur /rover/motor_cmd pour serial_bridge_node.
#
# CINÉMATIQUE DIFFÉRENTIELLE (tank drive) :
#   v_left  = linear - angular × WHEEL_BASE / 2
#   v_right = linear + angular × WHEEL_BASE / 2
#   Normalisé sur [-1..1] puis scalé à [-255..255] (PWM Arduino)
#
# MAPPING ROUES :
#   motor1 = Front Right  → v_right
#   motor2 = Front Left   → v_left
#   motor3 = Back Right   → v_right
#   motor4 = Back Left    → v_left
#
# TOPICS ÉCOUTÉS :
#   /rover/mode    String  — actif en 'race' ou 'autonomous' uniquement
#   /rover/cmd_vel Twist   — linear.x (m/s), angular.z (rad/s)
#
# TOPICS PUBLIÉS :
#   /rover/motor_cmd  Int32MultiArray [m1, m2, m3, m4]  (-255..255)
#
# SÉCURITÉ :
#   Timeout 500ms sans cmd_vel → publie zeros.
# ══════════════════════════════════════════════════════════════════════════════

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32MultiArray, String


def _clamp(v, lo, hi):
    return max(lo, min(hi, int(round(v))))


class MotorControllerNode(Node):

    WHEEL_BASE   = 0.25   # m — distance entre roues gauche et droite (à calibrer)
    MAX_SPEED_MS = 1.0    # m/s — vitesse max pour la normalisation
    TIMEOUT_SEC  = 0.5    # s — stop si pas de cmd_vel reçu depuis ce délai

    # Modes qui autorisent les moteurs de déplacement
    ACTIVE_MODES = {'race', 'autonomous', 'arm'}

    def __init__(self):
        super().__init__('motor_controller_node')

        self._current_mode  = 'idle'
        self._last_cmd_time = self.get_clock().now()

        self.create_subscription(String, '/rover/mode',    self._mode_cb, 10)
        self.create_subscription(Twist,  '/rover/cmd_vel', self._cmd_cb,  10)

        self._pub = self.create_publisher(Int32MultiArray, '/rover/motor_cmd', 10)

        self.create_timer(0.1, self._check_timeout)

        self.get_logger().info('motor_controller_node démarré')

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _mode_cb(self, msg: String):
        prev = self._current_mode
        self._current_mode = msg.data
        self.get_logger().info(f'Mode : {msg.data}')

        if prev in self.ACTIVE_MODES and msg.data not in self.ACTIVE_MODES:
            self._publish(0, 0)

    def _cmd_cb(self, msg: Twist):
        if self._current_mode not in self.ACTIVE_MODES:
            return

        self._last_cmd_time = self.get_clock().now()

        linear  = msg.linear.x
        angular = msg.angular.z

        v_left  = linear - angular * self.WHEEL_BASE / 2.0
        v_right = linear + angular * self.WHEEL_BASE / 2.0

        self._publish(v_left, v_right)

    # ── Timeout ───────────────────────────────────────────────────────────────

    def _check_timeout(self):
        if self._current_mode not in self.ACTIVE_MODES:
            return
        elapsed = (self.get_clock().now() - self._last_cmd_time).nanoseconds / 1e9
        if elapsed > self.TIMEOUT_SEC:
            self._publish(0, 0)

    # ── Publication ───────────────────────────────────────────────────────────

    def _publish(self, v_left: float, v_right: float):
        scale = 255.0 / self.MAX_SPEED_MS
        left  = _clamp(v_left  * scale, -255, 255)
        right = _clamp(v_right * scale, -255, 255)

        # motor1=FR, motor2=FL, motor3=BR, motor4=BL
        msg = Int32MultiArray()
        msg.data = [right, left, right, left]
        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MotorControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
