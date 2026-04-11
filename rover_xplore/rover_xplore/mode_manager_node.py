import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class ModeManagerNode(Node):
    """
    Nœud côté Raspberry Pi — reçoit le mode choisi par l'opérateur sur le PC.
    Subscribe à /rover/mode et agit en conséquence.
    Base de l'architecture : les futurs nœuds (autonomous_node, teleop_node)
    seront déclenchés depuis ici selon le mode actif.
    """

    def __init__(self):
        super().__init__('mode_manager_node')

        self.current_mode = None

        self.subscription = self.create_subscription(
            String,
            '/rover/mode',
            self.mode_callback,
            10
        )

        self.get_logger().info('mode_manager_node démarré — en attente du mode...')

    def mode_callback(self, msg):
        """Appelée à chaque message reçu sur /rover/mode."""
        mode = msg.data

        # Ignore si le mode n'a pas changé
        if mode == self.current_mode:
            return

        self.current_mode = mode

        if mode == 'autonomous':
            self.get_logger().info('═' * 40)
            self.get_logger().info('  MODE : AUTONOME')
            self.get_logger().info('  → navigation autonome activée')
            self.get_logger().info('═' * 40)
            # TODO : démarrer autonomous_node (machine d'état 5 phases)

        elif mode == 'race':
            self.get_logger().info('═' * 40)
            self.get_logger().info('  MODE : RACE (FPV)')
            self.get_logger().info('  → motor_controller_node actif sur /rover/cmd_vel')
            self.get_logger().info('═' * 40)

        elif mode == 'arm':
            self.get_logger().info('═' * 40)
            self.get_logger().info('  MODE : BRAS (ramassage)')
            self.get_logger().info('  → en attente des commandes bras')
            self.get_logger().info('═' * 40)
            # TODO : démarrer arm_node

        elif mode == 'idle':
            self.get_logger().info('═' * 40)
            self.get_logger().info('  MODE : IDLE')
            self.get_logger().info('  → rover en attente')
            self.get_logger().info('═' * 40)
            # motor_controller_node stoppe les moteurs dès réception du mode idle

        else:
            self.get_logger().warn(f'Mode inconnu reçu : "{mode}" — ignoré')


def main(args=None):
    rclpy.init(args=args)
    node = ModeManagerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
