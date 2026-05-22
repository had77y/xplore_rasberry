# ══════════════════════════════════════════════════════════════════════════════
# motor_controller_node.py — Raspberry Pi  (LifecycleNode)
#
# RÔLE : convertir les commandes de déplacement (Twist) en vitesses par roue
#         et les publier sur /rover/motor_cmd pour serial_bridge_node.
#
# CYCLE DE VIE (géré par mode_manager_node) :
#   on_configure  → crée le publisher /rover/motor_cmd
#   on_activate   → s'abonne à /rover/cmd_vel, timer sécurité
#   on_deactivate → publie zeros, détruit subscriptions + timer
#   on_cleanup    → détruit le publisher
#
# CINÉMATIQUE DIFFÉRENTIELLE (tank drive) :
#   v_left  = linear - angular × WHEEL_BASE / 2
#   v_right = linear + angular × WHEEL_BASE / 2
#
# CONTRÔLE EN BOUCLE OUVERTE :
#   PWM = clamp(v / MAX_SPEED_MS × 100, −100, 100)
#   Publié immédiatement à chaque Twist reçu.
#
# MAPPING ROUES :
#   motor1 = Front Right  → v_right
#   motor2 = Front Left   → v_left
#   motor3 = Back Right   → v_right
#   motor4 = Back Left    → v_left
#
# TOPICS ÉCOUTÉS (uniquement en Active) :
#   /rover/cmd_vel    geometry_msgs/Twist
#
# TOPICS PUBLIÉS :
#   /rover/motor_cmd  std_msgs/Int32MultiArray  [FR, FL, BR, BL]  (-100..100)
#
# SÉCURITÉ : timeout 500ms sans cmd_vel → PWM forcé à zéro.
# ══════════════════════════════════════════════════════════════════════════════

import rclpy
from geometry_msgs.msg import Twist
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn
from std_msgs.msg import Int32MultiArray


def _clamp(v: float, lo: int, hi: int) -> int:
    return max(lo, min(hi, int(round(v))))


WHEEL_BASE = 0.25    # m

# ── Physique (conservé pour référence / réactivation PID) ────────────────────
# from math import pi
# WHEEL_DIAMETER_MM = 120.0
# TICKS_PER_REV     = 1320
# MM_PER_TICK       = (pi * WHEEL_DIAMETER_MM) / TICKS_PER_REV   # ≈ 0.285 mm/tick
# DT                = 0.1     # s — période encodeurs (serial_bridge 10 Hz)
# MAX_ENC_TICKS     = 30.0   # Δticks/100ms à 100 PWM

# ── Gains PI (désactivé — boucle ouverte) ────────────────────────────────────
# KP             = 1.5
# KI             = 0.3
# INTEGRAL_CLAMP = 40.0


class MotorControllerNode(LifecycleNode):

    MAX_SPEED_MS = 1.0   # m/s — Twist linear.x = 1.0 correspond à cette vitesse
    TIMEOUT_SEC  = 0.5   # s — stop si pas de cmd_vel depuis ce délai

    def __init__(self):
        super().__init__('motor_controller_node')

        self._pub          = None
        self._sub_cmd_vel  = None
        self._timer_safety = None
        self._last_cmd_time = None

        # ── État PI (désactivé — conservé pour réactivation future) ──────────
        # self._tgt_left  = 0.0
        # self._tgt_right = 0.0
        # self._integral  = [0.0, 0.0]

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
        # ── Encodeurs (désactivés — boucle ouverte) ──────────────────────────
        # self._sub_encoders = self.create_subscription(
        #     Int32MultiArray, '/wheel_encoders', self._encoder_cb, 10
        # )
        self._timer_safety = self.create_timer(0.1, self._check_timeout)
        self.get_logger().info('motor_controller_node actif (boucle ouverte)')
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state):
        self._publish_pwm(0, 0)

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
        self._publish_pwm(0, 0)
        return TransitionCallbackReturn.SUCCESS

    def destroy_node(self):
        try:
            self._publish_pwm(0, 0)
        except Exception:
            pass
        super().destroy_node()

    # ── Réception Twist → PWM direct ─────────────────────────────────────────

    def _cmd_cb(self, msg: Twist):
        self._last_cmd_time = self.get_clock().now()
        v_left  = msg.linear.x - msg.angular.z * WHEEL_BASE / 2.0
        v_right = msg.linear.x + msg.angular.z * WHEEL_BASE / 2.0
        pwm_left  = _clamp(v_left  / self.MAX_SPEED_MS * 100.0, -100, 100)
        pwm_right = _clamp(v_right / self.MAX_SPEED_MS * 100.0, -100, 100)
        self._publish_pwm(pwm_left, pwm_right)

    # ── Boucle PI (désactivée — conservée pour réactivation future) ──────────

    # def _encoder_cb(self, msg: Int32MultiArray):
    #     if len(msg.data) < 4:
    #         return
    #     FR, FL, BR, BL = msg.data[:4]
    #     actual_left  = (FL + BL) / 2.0
    #     actual_right = (FR + BR) / 2.0
    #     pwm_left  = self._pi(0, self._tgt_left,  actual_left)
    #     pwm_right = self._pi(1, self._tgt_right, actual_right)
    #     self._publish_pwm(pwm_left, pwm_right)

    # def _pi(self, side: int, target: float, actual: float) -> int:
    #     error = target - actual
    #     if target == 0.0:
    #         self._integral[side] = 0.0
    #     else:
    #         self._integral[side] = max(
    #             -INTEGRAL_CLAMP,
    #             min(INTEGRAL_CLAMP, self._integral[side] + error)
    #         )
    #     return _clamp(KP * error + KI * self._integral[side], -100, 100)

    # def _reset_pi(self):
    #     self._integral = [0.0, 0.0]

    # ── Timeout sécurité ──────────────────────────────────────────────────────

    def _check_timeout(self):
        if self._last_cmd_time is None:
            return
        elapsed = (self.get_clock().now() - self._last_cmd_time).nanoseconds / 1e9
        if elapsed > self.TIMEOUT_SEC:
            self._publish_pwm(0, 0)

    # ── Publication ───────────────────────────────────────────────────────────

    def _publish_pwm(self, left: int, right: int):
        if self._pub is None:
            return
        msg = Int32MultiArray()
        msg.data = [left, -left, right, right]   # FR, FL, BR, BL
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
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
