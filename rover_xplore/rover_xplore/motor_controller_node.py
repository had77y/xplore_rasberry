# ══════════════════════════════════════════════════════════════════════════════
# motor_controller_node.py — Raspberry Pi  (LifecycleNode)
#
# RÔLE : convertir les commandes de déplacement (Twist) en vitesses par roue
#         et les publier sur /rover/motor_cmd pour serial_bridge_node.
#
# CYCLE DE VIE (géré par mode_manager_node) :
#   on_configure  → crée le publisher /rover/motor_cmd
#   on_activate   → s'abonne à /rover/cmd_vel, démarre le timer de sécurité
#   on_deactivate → publie zeros, détruit subscription + timer
#   on_cleanup    → détruit le publisher
#
# CINÉMATIQUE DIFFÉRENTIELLE (tank drive) :
#   v_left  = linear - angular × WHEEL_BASE / 2
#   v_right = linear + angular × WHEEL_BASE / 2
#   Scalé à [-255..255] (PWM Arduino)
#
# MAPPING ROUES :
#   motor1 = Front Right  → v_right
#   motor2 = Front Left   → v_left
#   motor3 = Back Right   → v_right
#   motor4 = Back Left    → v_left
#
# TOPICS ÉCOUTÉS (uniquement en Active) :
#   /rover/cmd_vel  geometry_msgs/Twist
#
# TOPICS PUBLIÉS :
#   /rover/motor_cmd  std_msgs/Int32MultiArray  [FR, FL, BR, BL]  (-255..255)
#
# SÉCURITÉ : timeout 500ms sans cmd_vel → publie zeros.
# ══════════════════════════════════════════════════════════════════════════════

import rclpy
from geometry_msgs.msg import Twist
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn
from std_msgs.msg import Int32MultiArray


def _clamp(v: float, lo: int, hi: int) -> int:
    return max(lo, min(hi, int(round(v))))


class MotorControllerNode(LifecycleNode):

    WHEEL_BASE   = 0.25   # m — distance entre roues gauche et droite (à calibrer)
    MAX_SPEED_MS = 1.0    # m/s — vitesse max (Twist linear.x = 1.0 → PWM 255)
    TIMEOUT_SEC  = 0.5    # s — stop si pas de cmd_vel depuis ce délai

    def __init__(self):
        super().__init__('motor_controller_node')

        self._pub           = None
        self._sub_cmd_vel   = None
        self._timer_safety  = None
        self._last_cmd_time = None

    # ── Lifecycle callbacks ───────────────────────────────────────────────────

    def on_configure(self, state):
        self._pub = self.create_publisher(Int32MultiArray, '/rover/motor_cmd', 10)
        self.get_logger().info('motor_controller_node configuré')
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state):
        self._last_cmd_time = self.get_clock().now()
        self._sub_cmd_vel  = self.create_subscription(
            Twist, '/rover/cmd_vel', self._cmd_cb, 10
        )
        self._timer_safety = self.create_timer(0.1, self._check_timeout)
        self.get_logger().info('motor_controller_node actif')
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state):
        self._publish_speeds(0.0, 0.0)

        if self._sub_cmd_vel is not None:
            self.destroy_subscription(self._sub_cmd_vel)
            self._sub_cmd_vel = None

        if self._timer_safety is not None:
            self.destroy_timer(self._timer_safety)
            self._timer_safety = None

        self.get_logger().info('motor_controller_node inactif — moteurs stoppés')
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state):
        if self._pub is not None:
            self.destroy_publisher(self._pub)
            self._pub = None
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state):
        self._publish_speeds(0.0, 0.0)
        return TransitionCallbackReturn.SUCCESS

    def destroy_node(self):
        # Appelé lors d'un Ctrl+C — on_shutdown n'est pas déclenché automatiquement
        self._publish_speeds(0.0, 0.0)
        super().destroy_node()

    # ── Traitement cmd_vel ────────────────────────────────────────────────────

    def _cmd_cb(self, msg: Twist):
        self._last_cmd_time = self.get_clock().now()
        linear  = msg.linear.x
        angular = msg.angular.z
        v_left  = linear - angular * self.WHEEL_BASE / 2.0
        v_right = linear + angular * self.WHEEL_BASE / 2.0
        self._publish_speeds(v_left, v_right)

    def _check_timeout(self):
        if self._last_cmd_time is None:
            return
        elapsed = (self.get_clock().now() - self._last_cmd_time).nanoseconds / 1e9
        if elapsed > self.TIMEOUT_SEC:
            self._publish_speeds(0.0, 0.0)

    # ── Publication ───────────────────────────────────────────────────────────

    def _publish_speeds(self, v_left: float, v_right: float):
        if self._pub is None:
            return
        scale = 255.0 / self.MAX_SPEED_MS
        left  = _clamp(v_left  * scale, -255, 255)
        right = _clamp(v_right * scale, -255, 255)
        msg = Int32MultiArray()
        msg.data = [right, left, right, left]  # FR, FL, BR, BL
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
