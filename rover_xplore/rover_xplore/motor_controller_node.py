# ──────────────────────────────────────────────────────────────────────────────
# FLUX DE DONNÉES
#
#   REÇOIT (1) : /rover/mode  (std_msgs/String)
#                → envoyé par controller_node (PC)
#                → si mode != "race" : stoppe immédiatement les moteurs
#
#   REÇOIT (2) : /rover/cmd_vel  (geometry_msgs/Twist)
#                → envoyé par controller_node (PC) en mode race
#                → linear.x  = vitesse avant/arrière (m/s)
#                → angular.z = vitesse de rotation (rad/s)
#
#   ENVOIE     : commande serial "L<val> R<val>\n" vers l'Arduino
#                → l'Arduino pilote les drivers moteur (PWM)
#                → val = entier [-100, 100] = % de vitesse max
#
#   SÉCURITÉ   : si aucun cmd_vel reçu depuis 500ms → stoppe les moteurs
#                (protège contre une perte de connexion réseau)
#
#   FUTUR      : quand encoder_node sera prêt, il publiera /wheel_odom
#                → ce node activera le PID par roue (voir code commenté)
# ──────────────────────────────────────────────────────────────────────────────

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

    # ── Paramètres PID — À CALIBRER sur le vrai hardware ────────────
    # KP = 1.0   # gain proportionnel
    # KI = 0.1   # gain intégral
    # KD = 0.01  # gain dérivé

    def __init__(self):
        super().__init__('motor_controller_node')

        self.current_mode = None

        # ── État PID par roue (décommenter quand encoder_node est prêt) ──
        # self.pid_left  = {'integral': 0.0, 'prev_error': 0.0, 'prev_time': None}
        # self.pid_right = {'integral': 0.0, 'prev_error': 0.0, 'prev_time': None}
        # self.speed_left  = 0.0  # vitesse mesurée roue gauche (m/s) — via /wheel_odom
        # self.speed_right = 0.0  # vitesse mesurée roue droite (m/s) — via /wheel_odom

        # ── Serial ───────────────────────────────────────────────────
        self.ser = None
        self._connect_serial()

        # ── Subscribers ──────────────────────────────────────────────
        # /rover/mode  → envoyé par controller_node (PC), gère le gating des moteurs
        self.create_subscription(String, '/rover/mode', self.mode_callback, 10)
        # /rover/cmd_vel → envoyé par controller_node (PC) uniquement en mode race
        self.create_subscription(Twist, '/rover/cmd_vel', self.cmd_callback, 10)

        # ── Safety timeout ───────────────────────────────────────────
        self.last_msg_time = self.get_clock().now()
        self.safety_timer = self.create_timer(0.1, self.check_timeout)

        self.get_logger().info('motor_controller_node démarré')

    # ── Mode ─────────────────────────────────────────────────────────

    def mode_callback(self, msg: String):
        # Reçoit le mode depuis controller_node (PC)
        # Si le rover sort du mode race → stoppe les moteurs immédiatement
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
        """
        Reçoit un Twist depuis controller_node (PC) via /rover/cmd_vel.
        Convertit en vitesses roues via cinématique différentielle.
        Envoie la commande à l'Arduino via serial.
        """
        # Gating : ignore les commandes si on n'est pas en mode race
        if self.current_mode != 'race':
            return

        self.last_msg_time = self.get_clock().now()

        linear  = msg.linear.x
        angular = msg.angular.z

        # Cinématique différentielle — vitesses cibles par roue (m/s)
        v_left  = linear - angular * self.WHEEL_BASE / 2.0
        v_right = linear + angular * self.WHEEL_BASE / 2.0

        # ── PID (décommenter quand encoder_node publie /wheel_odom) ─────
        # v_left  = self._pid(v_left,  self.speed_left,  self.pid_left)
        # v_right = self._pid(v_right, self.speed_right, self.pid_right)
        # ────────────────────────────────────────────────────────────────

        # Envoi à l'Arduino → drivers moteur → roues
        self._send_motors(v_left, v_right)

    # ── Envoi serial ─────────────────────────────────────────────────

    def _send_motors(self, v_left: float, v_right: float):
        """
        Convertit les vitesses (m/s) en pourcentage [-100, 100] et envoie à l'Arduino.
        Format : "L<val> R<val>\n"  ex: "L50 R50\n", "L-30 R30\n"
        L'Arduino interprète ce message et génère les signaux PWM pour les drivers moteur.
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

    # ── PID ──────────────────────────────────────────────────────────
    #
    # def _pid(self, setpoint: float, measured: float, state: dict) -> float:
    #     """Calcule la sortie PID pour une roue."""
    #     now = self.get_clock().now().nanoseconds / 1e9
    #     if state['prev_time'] is None:
    #         state['prev_time'] = now
    #         return setpoint
    #
    #     dt = now - state['prev_time']
    #     if dt <= 0:
    #         return setpoint
    #
    #     error = setpoint - measured
    #     state['integral']   += error * dt
    #     derivative           = (error - state['prev_error']) / dt
    #     state['prev_error']  = error
    #     state['prev_time']   = now
    #
    #     return self.KP * error + self.KI * state['integral'] + self.KD * derivative
    #
    # def _odom_callback(self, msg):
    #     """Reçoit les vitesses mesurées depuis encoder_node (/wheel_odom)."""
    #     # msg.twist.linear.x  → vitesse roue gauche
    #     # msg.twist.angular.z → vitesse roue droite
    #     self.speed_left  = msg.twist.linear.x
    #     self.speed_right = msg.twist.angular.z
    #
    # Abonnement à ajouter dans __init__ :
    # self.create_subscription(TwistStamped, '/wheel_odom', self._odom_callback, 10)

    # ── Safety timeout ───────────────────────────────────────────────

    def check_timeout(self):
        """
        Sécurité réseau : stoppe les moteurs si controller_node (PC) n'envoie plus
        de cmd_vel depuis TIMEOUT_SEC secondes (ex: perte WiFi).
        """
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
