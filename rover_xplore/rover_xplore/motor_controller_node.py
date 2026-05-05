# ══════════════════════════════════════════════════════════════════════════════
# motor_controller_node.py — Raspberry Pi
#
# RÔLE : recevoir les commandes de déplacement du PC et les transmettre
#         à l'Arduino via le port série, qui pilote les moteurs.
#
# PLACE DANS LE SYSTÈME :
#
#   [PC] controller_node / rover_gui
#         ├─ publie /rover/mode    → pour activer/désactiver ce node
#         └─ publie /rover/cmd_vel → commandes de vitesse (linear, angular)
#   [RPi] motor_controller_node ← CE FICHIER
#         └─ envoie via serial USB (/dev/ttyUSB0)
#   [Micro] Arduino
#         └─ reçoit "L<n> R<n>\n" → génère PWM → pilote les drivers moteur → roues
#
# TOPICS ÉCOUTÉS :
#   /rover/mode    (std_msgs/String)
#     → si mode != 'race' : stoppe les moteurs immédiatement
#     → gating de sécurité : empêche les moteurs de tourner dans le mauvais mode
#
#   /rover/cmd_vel (geometry_msgs/Twist)
#     → linear.x  = vitesse avant/arrière en m/s (positif = avant)
#     → angular.z = vitesse de rotation en rad/s (positif = gauche)
#     → traité SEULEMENT en mode 'race' (gating par mode)
#
# PROTOCOLE SERIAL VERS ARDUINO :
#   Format : "L<val> R<val>\n"
#   Exemples :
#     "L50 R50\n"   → avancer à 50% de la vitesse max
#     "L-30 R30\n"  → pivot gauche (roue gauche reculée, droite avancée)
#     "L0 R0\n"     → arrêt complet
#   val = entier [-100, 100] = pourcentage de la vitesse max
#   L'Arduino reçoit ce message et génère les PWM correspondants.
#
# CINÉMATIQUE DIFFÉRENTIELLE :
#   Le rover a deux groupes de roues (gauche et droite) comme un tank.
#   Pour avancer : v_gauche = v_droite (même vitesse)
#   Pour tourner  : v_gauche ≠ v_droite (différence de vitesse = rotation)
#   Formule :
#     v_gauche = linear - angular * WHEEL_BASE / 2
#     v_droite = linear + angular * WHEEL_BASE / 2
#   WHEEL_BASE = distance entre les deux groupes de roues (en m)
#
# SÉCURITÉ :
#   Si aucun cmd_vel reçu depuis 500ms → stoppe les moteurs.
#   Protège contre une perte de connexion WiFi (le rover ne continue pas en roue libre).
#
# ÉTAT ACTUEL (futur PID) :
#   Actuellement open-loop : on envoie une vitesse cible sans vérifier si les roues
#   tournent vraiment à cette vitesse. Le PID est prévu quand encoder_node sera prêt.
# ══════════════════════════════════════════════════════════════════════════════

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

    # ── Paramètres physiques — À CALIBRER avec l'équipe élec ──────────────────
    SERIAL_PORT   = '/dev/ttyUSB0'  # port série où l'Arduino est connecté au RPi
    BAUD_RATE     = 115200           # vitesse de communication série (bits/s)
    WHEEL_BASE    = 0.25             # m — distance entre roues gauche et droite (à mesurer)
    MAX_SPEED_MS  = 1.0              # m/s — vitesse max pour la normalisation
                                     # 1.0 = la valeur Twist devient directement le %
    TIMEOUT_SEC   = 0.5              # secondes — stop moteurs si pas de commande depuis ce délai

    def __init__(self):
        super().__init__('motor_controller_node')

        # Mode courant : utilisé pour le gating (n'agir que si mode == 'race')
        self.current_mode = None

        # ── Connexion au port série ────────────────────────────────────────────
        # L'Arduino est connecté via USB. Si le port est indisponible (tests sur VM),
        # on continue en mode "log uniquement" sans crasher.
        self.ser = None
        self._connect_serial()

        # ── Abonnements ───────────────────────────────────────────────────────
        # /rover/mode : envoyé par controller_node (PC) quand l'opérateur change de mode
        # QoS 10 = fiable, on ne veut pas rater un changement de mode
        self.create_subscription(String, '/rover/mode',     self.mode_callback, 10)
        # /rover/cmd_vel : envoyé par controller_node (PC) à ~10 Hz en mode race
        self.create_subscription(Twist,  '/rover/cmd_vel',  self.cmd_callback,  10)

        # ── Timer de sécurité (10 Hz) ─────────────────────────────────────────
        # Vérifie toutes les 100ms si une commande a été reçue récemment.
        # Si le WiFi coupe → controller_node arrête d'envoyer → ce timer stoppe les moteurs.
        self.last_msg_time = self.get_clock().now()
        self.safety_timer = self.create_timer(0.1, self.check_timeout)

        self.get_logger().info('motor_controller_node démarré')

    # ── Gestion du mode ───────────────────────────────────────────────────────

    def mode_callback(self, msg: String):
        """
        Reçoit le mode depuis /rover/mode (publié par le PC).
        Si on sort du mode 'race' → stoppe les moteurs immédiatement.
        Cela garantit que les moteurs s'arrêtent quand on passe en autonome,
        en bras, ou en idle.
        """
        mode = msg.data
        if mode == self.current_mode:
            return  # pas de changement → rien à faire
        self.current_mode = mode
        self.get_logger().info(f'Mode reçu : {mode}')
        if mode != 'race':
            # On sort du mode race → arrêt de sécurité immédiat
            self._send_motors(0.0, 0.0)

    # ── Connexion série ───────────────────────────────────────────────────────

    def _connect_serial(self):
        """
        Tente d'ouvrir le port série vers l'Arduino.
        En cas d'échec (port absent, VM, tests) → self.ser = None
        Les fonctions _send_motors() vérifient this.ser avant d'écrire.
        """
        try:
            self.ser = serial.Serial(self.SERIAL_PORT, self.BAUD_RATE, timeout=0.1)
            self.get_logger().info(f'Serial ouvert : {self.SERIAL_PORT} @ {self.BAUD_RATE}')
        except serial.SerialException as e:
            self.ser = None
            self.get_logger().warn(f'Serial indisponible ({e}) — mode log uniquement')

    # ── Réception commande de mouvement ──────────────────────────────────────

    def cmd_callback(self, msg: Twist):
        """
        Reçoit un Twist depuis /rover/cmd_vel (publié par controller_node ou rover_gui sur le PC).
        Le Twist contient :
          msg.linear.x  = vitesse avant/arrière (m/s)  ← W/S au clavier ou joystick
          msg.angular.z = vitesse de rotation (rad/s)  ← A/D au clavier ou joystick

        Étapes :
          1. Gating : ignore si mode != 'race'
          2. Cinématique différentielle → vitesses par roue
          3. Envoi série à l'Arduino
        """
        # GATING : si on n'est pas en mode race, on ignore toutes les commandes.
        # Cela empêche de déplacer le rover accidentellement en mode autonomous ou arm.
        if self.current_mode != 'race':
            return

        # Mise à jour du timer de sécurité : on a bien reçu une commande
        self.last_msg_time = self.get_clock().now()

        linear  = msg.linear.x   # positif = avancer, négatif = reculer
        angular = msg.angular.z  # positif = tourner gauche, négatif = tourner droite

        # Cinématique différentielle (modèle "tank drive")
        # Si on avance tout droit : linear=1, angular=0 → v_left=1, v_right=1 ✓
        # Si on pivote gauche    : linear=0, angular=1 → v_left=-0.125, v_right=0.125 ✓
        # Si on arc avant gauche : linear=1, angular=1 → v_left=0.875, v_right=1.125 ✓
        v_left  = linear - angular * self.WHEEL_BASE / 2.0
        v_right = linear + angular * self.WHEEL_BASE / 2.0

        # Envoi à l'Arduino
        self._send_motors(v_left, v_right)

    # ── Envoi série ───────────────────────────────────────────────────────────

    def _send_motors(self, v_left: float, v_right: float):
        """
        Convertit les vitesses (m/s) en pourcentage [-100, 100] et envoie à l'Arduino.

        Normalisation : val = (v / MAX_SPEED_MS) × 100
        Clamp à [-100, 100] pour ne pas dépasser les limites.

        Exemple : v_left=0.5 m/s, MAX=1.0 → left_pct = 50
        Format envoyé : "L50 R50\n"

        L'Arduino parse cette chaîne, extrait les valeurs L et R,
        et les convertit en signaux PWM pour les drivers moteur.
        """
        left_pct  = int(max(-100, min(100, (v_left  / self.MAX_SPEED_MS) * 100)))
        right_pct = int(max(-100, min(100, (v_right / self.MAX_SPEED_MS) * 100)))

        cmd = f'L{left_pct} R{right_pct}\n'
        self.get_logger().info(f'Moteurs → {cmd.strip()}')

        if self.ser and self.ser.is_open:
            try:
                self.ser.write(cmd.encode())
            except serial.SerialException as e:
                # Erreur de communication → connexion perdue, on remet à None
                # pour éviter des erreurs répétées
                self.get_logger().error(f'Erreur serial : {e}')
                self.ser = None

    # ── Timer de sécurité ─────────────────────────────────────────────────────

    def check_timeout(self):
        """
        Appelée toutes les 100ms.
        Si controller_node (PC) n'envoie plus de commandes depuis TIMEOUT_SEC=0.5s
        → stoppe les moteurs.

        Scénarios protégés :
          - Perte du signal WiFi
          - Crash du processus controller_node sur le PC
          - L'opérateur ferme l'application PC sans appuyer sur Stop
        """
        elapsed = (self.get_clock().now() - self.last_msg_time).nanoseconds / 1e9
        if elapsed > self.TIMEOUT_SEC:
            self._send_motors(0.0, 0.0)

    # ── Nettoyage ─────────────────────────────────────────────────────────────

    def destroy_node(self):
        """Arrête les moteurs et ferme le port série avant de quitter."""
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
