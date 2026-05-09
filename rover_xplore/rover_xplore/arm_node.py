# ══════════════════════════════════════════════════════════════════════════════
# arm_node.py — Raspberry Pi
#
# RÔLE : recevoir les commandes bras depuis le PC et les convertir en
#         commandes servo/stepper pour serial_bridge_node.
#
# MAPPING arm_cmd → struct Arduino :
#   z   × speed → stepper   (axe Z bras, up/down)           [-100..100]
#   y   × speed → servo_1   (flip/unflip)                   [-100..100]
#   pince × speed → servo_2 = servo_3  (open/close pinces)  [-100..100]
#   bin_dir × speed → servo_4  (benne haut/bas)             [-100..100]
#   dump > 0.5  → servo_4 = 100 (position de vidage complète)
#
# TOPICS ÉCOUTÉS :
#   /rover/arm_cmd  Float32MultiArray [z, y, pince, speed, dump, bin_dir]
#   /rover/mode     String — gating, actif uniquement en mode 'arm'
#
# TOPICS PUBLIÉS :
#   /rover/arm_serial_cmd  Int32MultiArray [servo1, servo2, servo3, servo4, stepper]
# ══════════════════════════════════════════════════════════════════════════════

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Int32MultiArray, String


def _clamp(v, lo, hi):
    return max(lo, min(hi, int(round(v))))


class ArmNode(Node):

    def __init__(self):
        super().__init__('arm_node')

        self._current_mode = 'idle'

        self.create_subscription(Float32MultiArray, '/rover/arm_cmd', self._arm_cmd_cb, 10)
        self.create_subscription(String,            '/rover/mode',    self._mode_cb,    10)

        self._pub = self.create_publisher(Int32MultiArray, '/rover/arm_serial_cmd', 10)

        self.get_logger().info('arm_node démarré')

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _mode_cb(self, msg: String):
        prev = self._current_mode
        self._current_mode = msg.data

        # Sortie du mode arm → zeros immédiats
        if prev == 'arm' and msg.data != 'arm':
            self._publish(0, 0, 0, 0, 0)

    def _arm_cmd_cb(self, msg: Float32MultiArray):
        if self._current_mode != 'arm':
            return
        if len(msg.data) < 6:
            return

        z, y, pince, speed, dump, bin_dir = msg.data[:6]

        stepper = _clamp(z     * speed * 100.0, -100, 100)
        s1      = _clamp(y     * speed * 100.0, -100, 100)  # flip/unflip
        s23     = _clamp(pince * speed * 100.0, -100, 100)  # open/close (gauche = droite)

        # Benne : position de vidage prioritaire sur la direction normale
        if dump > 0.5:
            s4 = 100
        else:
            s4 = _clamp(bin_dir * speed * 100.0, -100, 100)

        self._publish(s1, s23, s23, s4, stepper)

    # ── Publication ───────────────────────────────────────────────────────────

    def _publish(self, s1, s2, s3, s4, stepper):
        out = Int32MultiArray()
        out.data = [s1, s2, s3, s4, stepper]
        self._pub.publish(out)


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
