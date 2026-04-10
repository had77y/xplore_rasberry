import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class TeleopReceiverNode(Node):
    """
    Nœud côté Raspberry Pi — reçoit les commandes de téléopération depuis le PC.
    Subscribe à /rover/cmd_vel et log la commande reçue.
    Stoppe automatiquement si aucune commande reçue depuis 500ms (sécurité réseau).
    TODO : remplacer les logs par le contrôle réel des moteurs.
    """

    TIMEOUT_SEC = 0.5  # stoppe si pas de commande depuis 500ms

    def __init__(self):
        super().__init__('teleop_receiver_node')

        self.subscription = self.create_subscription(
            Twist,
            '/rover/cmd_vel',
            self.cmd_callback,
            10
        )

        # Timer de sécurité — vérifie le timeout toutes les 100ms
        self.last_msg_time = self.get_clock().now()
        self.is_moving = False
        self.safety_timer = self.create_timer(0.1, self.check_timeout)

        self.get_logger().info('teleop_receiver_node démarré — en attente de commandes...')

    def cmd_callback(self, msg):
        """Reçoit une commande Twist et log le mouvement correspondant."""
        self.last_msg_time = self.get_clock().now()

        linear = msg.linear.x
        angular = msg.angular.z

        # Log lisible selon la combinaison linear/angular
        if linear > 0 and angular == 0:
            self.get_logger().info('→ AVANCER')
        elif linear < 0 and angular == 0:
            self.get_logger().info('→ RECULER')
        elif linear == 0 and angular > 0:
            self.get_logger().info('→ PIVOT GAUCHE')
        elif linear == 0 and angular < 0:
            self.get_logger().info('→ PIVOT DROITE')
        elif linear > 0 and angular > 0:
            self.get_logger().info('→ VIRAGE GAUCHE (arc avant)')
        elif linear > 0 and angular < 0:
            self.get_logger().info('→ VIRAGE DROITE (arc avant)')
        elif linear < 0 and angular > 0:
            self.get_logger().info('→ VIRAGE GAUCHE (arc arrière)')
        elif linear < 0 and angular < 0:
            self.get_logger().info('→ VIRAGE DROITE (arc arrière)')
        elif linear == 0 and angular == 0:
            if self.is_moving:
                self.get_logger().info('→ STOP')
                self.is_moving = False
            return

        self.is_moving = True

    def check_timeout(self):
        """Stoppe le rover si aucune commande reçue depuis TIMEOUT_SEC."""
        elapsed = (self.get_clock().now() - self.last_msg_time).nanoseconds / 1e9
        if elapsed > self.TIMEOUT_SEC and self.is_moving:
            self.get_logger().warn('Timeout commande — STOP sécurité')
            self.is_moving = False
            # TODO : couper les moteurs ici


def main(args=None):
    rclpy.init(args=args)
    node = TeleopReceiverNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
