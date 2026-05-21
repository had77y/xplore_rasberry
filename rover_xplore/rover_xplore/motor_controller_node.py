# ══════════════════════════════════════════════════════════════════════════════
# motor_controller_node.py — Raspberry Pi  (LifecycleNode)
#
# RÔLE : convertir les commandes de déplacement (Twist) en vitesses par roue
#         et les publier sur /rover/motor_cmd pour serial_bridge_node.
#
# CYCLE DE VIE (géré par mode_manager_node) :
#   on_configure  → crée le publisher /rover/motor_cmd
#   on_activate   → s'abonne à /rover/cmd_vel + /wheel_encoders, timer sécurité
#   on_deactivate → publie zeros, détruit subscriptions + timer, reset PI
#   on_cleanup    → détruit le publisher
#
# CINÉMATIQUE DIFFÉRENTIELLE (tank drive) :
#   v_left  = linear - angular × WHEEL_BASE / 2
#   v_right = linear + angular × WHEEL_BASE / 2
#
# CONTRÔLE PI FERMÉ PAR ROUE :
#   Setpoint : Δticks/100ms calculé depuis le Twist reçu.
#   Feedback : Δticks/100ms mesuré par l'ESP32 sur /wheel_encoders.
#   Sortie   : PWM [-100..100] vers /rover/motor_cmd.
#   Les moteurs ne sont commandés qu'à la réception d'encodeurs (10 Hz).
#   Si les encodeurs ne répondent plus → serial_bridge coupe les moteurs
#   après MOTOR_TIMEOUT_S (fail-safe matériel).
#
# MAPPING ROUES :
#   motor1 = Front Right  → v_right
#   motor2 = Front Left   → v_left
#   motor3 = Back Right   → v_right
#   motor4 = Back Left    → v_left
#
# TOPICS ÉCOUTÉS (uniquement en Active) :
#   /rover/cmd_vel    geometry_msgs/Twist
#   /wheel_encoders   std_msgs/Int32MultiArray  [FR, FL, BR, BL]  Δticks/100ms
#
# TOPICS PUBLIÉS :
#   /rover/motor_cmd  std_msgs/Int32MultiArray  [FR, FL, BR, BL]  (-100..100)
#
# SÉCURITÉ : timeout 500ms sans cmd_vel → setpoints à zéro + reset PI.
# ══════════════════════════════════════════════════════════════════════════════

from math import pi

import rclpy
from geometry_msgs.msg import Twist
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn
from std_msgs.msg import Int32MultiArray


def _clamp(v: float, lo: int, hi: int) -> int:
    return max(lo, min(hi, int(round(v))))


# ── Physique — à calibrer en même temps que odometry_node ────────────────────
WHEEL_DIAMETER_MM = 120.0
TICKS_PER_REV     = 1320
MM_PER_TICK       = (pi * WHEEL_DIAMETER_MM) / TICKS_PER_REV   # ≈ 0.285 mm/tick
WHEEL_BASE        = 0.25    # m
DT                = 0.1     # s — période encodeurs (serial_bridge 10 Hz)

MAX_ENC_TICKS  = 30.0   # Δticks/100ms à 100 PWM — à affiner par mesure

# ── Gains PI — à calibrer sur le vrai rover ───────────────────────────────────
# Unités : erreur et intégrateur en Δticks/100ms, sortie en PWM [-100..100]
KP             = 1.5
KI             = 0.3
INTEGRAL_CLAMP = 40.0   # anti-windup


class MotorControllerNode(LifecycleNode):

    MAX_SPEED_MS = 1.0   # m/s — Twist linear.x = 1.0 correspond à cette vitesse
    TIMEOUT_SEC  = 0.5   # s — stop si pas de cmd_vel depuis ce délai

    def __init__(self):
        super().__init__('motor_controller_node')

        self._pub          = None
        self._sub_cmd_vel  = None
        self._sub_encoders = None
        self._timer_safety = None
        self._last_cmd_time = None

        # Setpoints en Δticks/100ms (mis à jour par _cmd_cb)
        self._tgt_left  = 0.0
        self._tgt_right = 0.0

        # État PI : [0]=gauche, [1]=droite
        self._integral = [0.0, 0.0]

    # ── Lifecycle callbacks ───────────────────────────────────────────────────

    def on_configure(self, state):
        self._pub = self.create_publisher(Int32MultiArray, '/rover/motor_cmd', 10)
        self.get_logger().info('motor_controller_node configuré')
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state):
        self._last_cmd_time = self.get_clock().now()
        self._reset_pi()
        self._sub_cmd_vel  = self.create_subscription(
            Twist, '/rover/cmd_vel', self._cmd_cb, 10
        )
        self._sub_encoders = self.create_subscription(
            Int32MultiArray, '/wheel_encoders', self._encoder_cb, 10
        )
        self._timer_safety = self.create_timer(0.1, self._check_timeout)
        self.get_logger().info('motor_controller_node actif')
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state):
        self._tgt_left = self._tgt_right = 0.0
        self._reset_pi()
        self._publish_pwm(0, 0)

        if self._sub_cmd_vel is not None:
            self.destroy_subscription(self._sub_cmd_vel)
            self._sub_cmd_vel = None
        if self._sub_encoders is not None:
            self.destroy_subscription(self._sub_encoders)
            self._sub_encoders = None
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

    # ── Réception Twist ───────────────────────────────────────────────────────

    def _cmd_cb(self, msg: Twist):
        self._last_cmd_time = self.get_clock().now()
        v_left  = msg.linear.x - msg.angular.z * WHEEL_BASE / 2.0   # m/s
        v_right = msg.linear.x + msg.angular.z * WHEEL_BASE / 2.0
        # Convertir m/s → Δticks/100ms, borné au max physique
        k = 1000.0 * DT / MM_PER_TICK
        self._tgt_left  = max(-MAX_ENC_TICKS, min(MAX_ENC_TICKS, v_left  * k))
        self._tgt_right = max(-MAX_ENC_TICKS, min(MAX_ENC_TICKS, v_right * k))

    # ── Boucle PI (appelée à chaque réception encodeurs, 10 Hz) ──────────────

    def _encoder_cb(self, msg: Int32MultiArray):
        if len(msg.data) < 4:
            return
        FR, FL, BR, BL = msg.data[:4]
        actual_left  = (FL + BL) / 2.0   # Δticks/100ms côté gauche
        actual_right = (FR + BR) / 2.0   # Δticks/100ms côté droit

        pwm_left  = self._pi(0, self._tgt_left,  actual_left)
        pwm_right = self._pi(1, self._tgt_right, actual_right)
        self._publish_pwm(pwm_left, pwm_right)

    def _pi(self, side: int, target: float, actual: float) -> int:
        error = target - actual

        # Vider l'intégrateur à l'arrêt pour éviter le windup au redémarrage
        if target == 0.0:
            self._integral[side] = 0.0
        else:
            self._integral[side] = max(
                -INTEGRAL_CLAMP,
                min(INTEGRAL_CLAMP, self._integral[side] + error)
            )

        return _clamp(KP * error + KI * self._integral[side], -100, 100)

    def _reset_pi(self):
        self._integral = [0.0, 0.0]

    # ── Timeout sécurité ──────────────────────────────────────────────────────

    def _check_timeout(self):
        if self._last_cmd_time is None:
            return
        elapsed = (self.get_clock().now() - self._last_cmd_time).nanoseconds / 1e9
        if elapsed > self.TIMEOUT_SEC:
            self._tgt_left = self._tgt_right = 0.0
            self._reset_pi()

    # ── Publication ───────────────────────────────────────────────────────────

    def _publish_pwm(self, left: int, right: int):
        if self._pub is None:
            return
        msg = Int32MultiArray()
        msg.data = [right, left, right, left]   # FR, FL, BR, BL
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
