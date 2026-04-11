import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import serial


class MotorControllerNode(Node):
    """
    Reçoit /rover/cmd_vel (Twist) et envoie les commandes moteur à l'Arduino via serial.

    Conversion cinématique différentielle :
        v_left  = linear - angular * WHEEL_BASE / 2
        v_right = linear + angular * WHEEL_BASE / 2

    Protocole serial vers Arduino : "L<val> R<val>\n"
        val = entier [-100, 100] représentant le % de vitesse max

    TODO — PID :
        Actuellement open-loop (pas de retour encodeur).
        Quand les encodeurs seront câblés, ajouter ici :
          - un PID par roue (setpoint = v_cible, mesure = v_encodeur)
          - publier les vitesses mesurées sur /wheel_odom
    """

    # ── Paramètres — À CALIBRER avec l'équipe élec ──────────────────
    SERIAL_PORT   = '/dev/ttyUSB0'
    BAUD_RATE     = 115200
    WHEEL_BASE    = 0.25   # m — distance entre roues gauche et droite
    MAX_SPEED_MS  = 1.0    # m/s — À CALIBRER avec encodeurs ; 1.0 = identité pour téléop clavier
    TIMEOUT_SEC   = 0.5    # s  — stop si pas de commande depuis ce délai

    def __init__(self):
        super().__init__('motor_controller_node')

        self.current_mode = None

        # ── Serial ───────────────────────────────────────────────────
        self.ser = None
        self._connect_serial()

        # ── Subscribers ──────────────────────────────────────────────
        self.create_subscription(String, '/rover/mode', self.mode_callback, 10)
        self.create_subscription(Twist, '/rover/cmd_vel', self.cmd_callback, 10)

        # ── Safety timeout ───────────────────────────────────────────
        self.last_msg_time = self.get_clock().now()
        self.safety_timer = self.create_timer(0.1, self.check_timeout)

        self.get_logger().info('motor_controller_node démarré')

    # ── Mode ─────────────────────────────────────────────────────────

    def mode_callback(self, msg: String):
        mode = msg.data
        if mode == self.current_mode:
            return
        self.current_mode = mode
        self.get_logger().info(f'Mode reçu : {mode}')
        if mode != 'race':
            self._send_motors(0.0, 0.0)

    # ── Connexion serial ─────────────────────────────────────────────

    def _connect_serial(self):
        """Tente d'ouvrir le port serial. Log un warning si indisponible."""
        try:
            self.ser = serial.Serial(self.SERIAL_PORT, self.BAUD_RATE, timeout=0.1)
            self.get_logger().info(f'Serial ouvert : {self.SERIAL_PORT} @ {self.BAUD_RATE}')
        except serial.SerialException as e:
            self.ser = None
            self.get_logger().warn(f'Serial indisponible ({e}) — mode log uniquement')

    # ── Callback commande ────────────────────────────────────────────

    def cmd_callback(self, msg: Twist):
        """Reçoit Twist, convertit en vitesses roues, envoie à l'Arduino."""
        if self.current_mode != 'race':
            return

        self.last_msg_time = self.get_clock().now()

        linear  = msg.linear.x
        angular = msg.angular.z

        # Cinématique différentielle
        v_left  = linear - angular * self.WHEEL_BASE / 2.0
        v_right = linear + angular * self.WHEEL_BASE / 2.0

        # TODO PID : ici on enverrait v_left/v_right comme setpoints au PID
        #            et on lirait les vitesses mesurées par les encodeurs.
        #            Pour l'instant, conversion directe en %.

        self._send_motors(v_left, v_right)

    # ── Envoi serial ─────────────────────────────────────────────────

    def _send_motors(self, v_left: float, v_right: float):
        """
        Convertit les vitesses (m/s) en pourcentage [-100, 100] et envoie à l'Arduino.
        Format : "L<val> R<val>\n"  ex: "L50 R50\n", "L-30 R30\n"
        """
        left_pct  = int(max(-100, min(100, (v_left  / self.MAX_SPEED_MS) * 100)))
        right_pct = int(max(-100, min(100, (v_right / self.MAX_SPEED_MS) * 100)))

        cmd = f'L{left_pct} R{right_pct}\n'
        self.get_logger().info(f'Moteurs → {cmd.strip()}')

        if self.ser and self.ser.is_open:
            try:
                self.ser.write(cmd.encode())
            except serial.SerialException as e:
                self.get_logger().error(f'Erreur serial : {e}')
                self.ser = None

    # ── Safety timeout ───────────────────────────────────────────────

    def check_timeout(self):
        """Stoppe les moteurs si aucune commande reçue depuis TIMEOUT_SEC."""
        elapsed = (self.get_clock().now() - self.last_msg_time).nanoseconds / 1e9
        if elapsed > self.TIMEOUT_SEC:
            self._send_motors(0.0, 0.0)

    # ── Cleanup ──────────────────────────────────────────────────────

    def destroy_node(self):
        self._send_motors(0.0, 0.0)
        if self.ser and self.ser.is_open:
            self.ser.close()
        super().destroy_node()


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
