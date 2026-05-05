# ══════════════════════════════════════════════════════════════════════════════
# mode_manager_node.py — Raspberry Pi
#
# RÔLE : recevoir le mode choisi par l'opérateur sur le PC et dispatcher
#         vers les sous-systèmes correspondants sur le rover.
#
# PLACE DANS LE SYSTÈME :
#
#   [PC] controller_node.py  (ou rover_gui.py)
#         ↓  publie /rover/mode  (String : "autonomous" / "race" / "arm" / "idle")
#         ↓  via WiFi → ROS2 DDS
#   [RPi] mode_manager_node.py   ← CE FICHIER
#         ├─ loggue le changement de mode
#         └─ (futur) démarre/arrête les sous-nœuds selon le mode
#
# CE NODE ÉCOUTE :
#   /rover/mode  (std_msgs/String)
#
# CE NODE NE PUBLIE RIEN (pour l'instant — c'est un dispatcher passif).
#
# LES AUTRES NODES QUI ÉCOUTENT AUSSI /rover/mode DIRECTEMENT :
#   - camera_node.py        → active/désactive la capture selon le mode
#   - aruco_node.py         → active la détection ArUco seulement en 'autonomous'
#   - motor_controller_node → stoppe les moteurs si mode != 'race'
#
# MODES POSSIBLES :
#   "autonomous" → le rover navigue seul (futur : lancer autonomous_node)
#   "race"       → téléopération FPV (motor_controller_node reçoit /rover/cmd_vel)
#   "arm"        → contrôle du bras robotique (futur : lancer arm_node)
#   "idle"       → état d'attente, tous les actionneurs stoppés
# ══════════════════════════════════════════════════════════════════════════════

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class ModeManagerNode(Node):
    """
    Nœud côté Raspberry Pi — reçoit le mode choisi par l'opérateur sur le PC.
    Subscribe à /rover/mode et agit en conséquence.
    Base de l'architecture : les futurs nœuds (autonomous_node, arm_node)
    seront déclenchés depuis ici selon le mode actif.
    """

    def __init__(self):
        super().__init__('mode_manager_node')

        # On mémorise le mode courant pour ignorer les messages dupliqués
        # (le PC peut republier le même mode plusieurs fois sans changement)
        self.current_mode = None

        # Abonnement à /rover/mode.
        # Ce topic est publié par controller_node (terminal PC) ou rover_gui.py (GUI PC).
        # QoS par défaut (10) = fiable, garde les 10 derniers messages.
        self.subscription = self.create_subscription(
            String,
            '/rover/mode',
            self.mode_callback,
            10
        )

        self.get_logger().info('mode_manager_node démarré — en attente du mode...')

    def mode_callback(self, msg):
        """
        Appelée automatiquement par ROS2 à chaque message reçu sur /rover/mode.
        Le message vient du PC (controller_node ou rover_gui.py).

        msg.data est une string parmi : "autonomous", "race", "arm", "idle"
        """
        mode = msg.data

        # Si le mode n'a pas changé → rien à faire.
        # Sans ce guard, on logguerait indéfiniment si le PC republié en boucle.
        if mode == self.current_mode:
            return

        self.current_mode = mode

        if mode == 'autonomous':
            self.get_logger().info('═' * 40)
            self.get_logger().info('  MODE : AUTONOME')
            self.get_logger().info('  → navigation autonome activée')
            self.get_logger().info('═' * 40)
            # TODO : démarrer autonomous_node (machine d'état 5 phases définie dans auto_plan.md)
            # autonomous_node lira /aruco_detected (depuis aruco_node) + /distances (ultrasoniques)
            # et publiera /rover/cmd_vel pour déplacer le rover

        elif mode == 'race':
            self.get_logger().info('═' * 40)
            self.get_logger().info('  MODE : RACE (FPV)')
            self.get_logger().info('  → motor_controller_node actif sur /rover/cmd_vel')
            self.get_logger().info('═' * 40)
            # motor_controller_node (ce repo) écoute /rover/cmd_vel envoyé par controller_node (PC)
            # et transmet les commandes à l'Arduino via le port série (/dev/ttyUSB0)

        elif mode == 'arm':
            self.get_logger().info('═' * 40)
            self.get_logger().info('  MODE : BRAS (ramassage)')
            self.get_logger().info('  → en attente des commandes bras')
            self.get_logger().info('═' * 40)
            # TODO : démarrer arm_node
            # arm_node recevra les commandes depuis controller_node (PC) sur /rover/arm_cmd

        elif mode == 'idle':
            self.get_logger().info('═' * 40)
            self.get_logger().info('  MODE : IDLE')
            self.get_logger().info('  → rover en attente')
            self.get_logger().info('═' * 40)
            # motor_controller_node stoppe les moteurs dès réception du mode idle
            # (il écoute /rover/mode directement et coupe les moteurs si mode != 'race')

        else:
            # Mode inconnu → probablement une faute de frappe ou une version obsolète du PC
            self.get_logger().warn(f'Mode inconnu reçu : "{mode}" — ignoré')


def main(args=None):
    rclpy.init(args=args)
    node = ModeManagerNode()
    # spin() bloque ici et appelle mode_callback() à chaque message /rover/mode reçu
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
