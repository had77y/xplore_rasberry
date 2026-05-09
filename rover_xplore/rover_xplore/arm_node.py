# ══════════════════════════════════════════════════════════════════════════════
# arm_node.py — Raspberry Pi  (LifecycleNode)
#
# RÔLE : convertir les commandes bras reçues du PC en valeurs servo/stepper
#         et les publier sur /rover/arm_serial_cmd pour serial_bridge_node.
#
# CYCLE DE VIE (géré par mode_manager_node) :
#   on_configure  → crée le publisher /rover/arm_serial_cmd
#   on_activate   → s'abonne à /rover/arm_cmd
#   on_deactivate → publie zeros, détruit subscription
#   on_cleanup    → détruit le publisher
#
# MAPPING arm_cmd → struct Arduino :
#   z     × speed → stepper   (axe Z, up/down)              [-100..100]
#   y     × speed → servo_1   (flip/unflip)                  [-100..100]
#   pince × speed → servo_2 = servo_3  (open/close pinces)   [-100..100]
#   bin_dir × speed → servo_4  (benne haut/bas)              [-100..100]
#   dump > 0.5  → servo_4 = 100  (position de vidage forcée)
#
# TOPICS ÉCOUTÉS (uniquement en Active) :
#   /rover/arm_cmd  std_msgs/Float32MultiArray  [z, y, pince, speed, dump, bin_dir]
#
# TOPICS PUBLIÉS :
#   /rover/arm_serial_cmd  std_msgs/Int32MultiArray  [s1, s2, s3, s4, stepper]
# ══════════════════════════════════════════════════════════════════════════════

import rclpy
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn
from std_msgs.msg import Float32MultiArray, Int32MultiArray


def _clamp(v: float, lo: int, hi: int) -> int:
    return max(lo, min(hi, int(round(v))))


class ArmNode(LifecycleNode):

    def __init__(self):
        super().__init__('arm_node')

        self._pub         = None
        self._sub_arm_cmd = None

    # ── Lifecycle callbacks ───────────────────────────────────────────────────

    def on_configure(self, state):
        self._pub = self.create_publisher(Int32MultiArray, '/rover/arm_serial_cmd', 10)
        self.get_logger().info('arm_node configuré')
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state):
        self._sub_arm_cmd = self.create_subscription(
            Float32MultiArray, '/rover/arm_cmd', self._arm_cmd_cb, 10
        )
        self.get_logger().info('arm_node actif')
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state):
        if self._sub_arm_cmd is not None:
            self.destroy_subscription(self._sub_arm_cmd)
            self._sub_arm_cmd = None

        self._publish(0, 0, 0, 0, 0)
        self.get_logger().info('arm_node inactif — bras stoppé')
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state):
        if self._pub is not None:
            self.destroy_publisher(self._pub)
            self._pub = None
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state):
        self._publish(0, 0, 0, 0, 0)
        return TransitionCallbackReturn.SUCCESS

    def destroy_node(self):
        # Appelé lors d'un Ctrl+C — on_shutdown n'est pas déclenché automatiquement
        self._publish(0, 0, 0, 0, 0)
        super().destroy_node()

    # ── Traitement arm_cmd ────────────────────────────────────────────────────

    def _arm_cmd_cb(self, msg: Float32MultiArray):
        if len(msg.data) < 6:
            return

        z, y, pince, speed, dump, bin_dir = msg.data[:6]

        stepper = _clamp(z     * speed * 100.0, -100, 100)
        s1      = _clamp(y     * speed * 100.0, -100, 100)
        s23     = _clamp(pince * speed * 100.0, -100, 100)
        s4      = 100 if dump > 0.5 else _clamp(bin_dir * speed * 100.0, -100, 100)

        self._publish(s1, s23, s23, s4, stepper)

    # ── Publication ───────────────────────────────────────────────────────────

    def _publish(self, s1: int, s2: int, s3: int, s4: int, stepper: int):
        if self._pub is None:
            return
        msg = Int32MultiArray()
        msg.data = [s1, s2, s3, s4, stepper]
        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ArmNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
