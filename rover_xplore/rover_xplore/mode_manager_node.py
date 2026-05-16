# ══════════════════════════════════════════════════════════════════════════════
# mode_manager_node.py — Raspberry Pi
#
# RÔLE : orchestrateur unique de l'architecture lifecycle.
#         Seul node qui connaît la logique "qui fait quoi en quel mode".
#         Les nodes gérés (motor_controller, arm_node, aruco_node) n'ont aucune
#         connaissance du mode — c'est ce node qui les active/désactive via les
#         services ROS2 lifecycle standard.
#
# FLUX :
#   [PC] rover_gui → /rover/mode (String)
#   [RPi] mode_manager_node → appelle /<node>/change_state (lifecycle service)
#
# SÉQUENCE AU DÉMARRAGE :
#   1. Attend que les services lifecycle de chaque node géré soient disponibles
#   2. Configure tous les nodes (Unconfigured → Inactive)
#   3. Attend les modes envoyés par le PC
#
# TRANSITIONS PAR MODE :
#   idle       : motor=Inactive  arm=Inactive  aruco=Inactive
#   race       : motor=Active    arm=Inactive  aruco=Inactive
#   arm        : motor=Active    arm=Active    aruco=Inactive
#   autonomous : motor=Active    arm=Active    aruco=Active
#
# ORDRE DES TRANSITIONS :
#   Désactivations d'abord (les nodes publient leurs zeros) → activations ensuite.
#   Garantit qu'aucun acteur non voulu ne reste actif pendant le changement.
# ══════════════════════════════════════════════════════════════════════════════

import queue
import threading
import time

import rclpy
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState
from rclpy.node import Node
from std_msgs.msg import String

# Table de vérité : pour chaque mode, quel node doit être actif ?
_MODE_MAP: dict[str, dict[str, bool]] = {
    'idle':       {'motor_controller_node': False, 'arm_node': False, 'aruco_node': False, 'autonomous_node': False},
    'race':       {'motor_controller_node': True,  'arm_node': False, 'aruco_node': False, 'autonomous_node': False},
    'arm':        {'motor_controller_node': True,  'arm_node': True,  'aruco_node': False, 'autonomous_node': False},
    'autonomous': {'motor_controller_node': True,  'arm_node': False, 'aruco_node': True,  'autonomous_node': True},
}

_MANAGED_NODES = list(next(iter(_MODE_MAP.values())).keys())

# Délai d'attente au démarrage avant de configurer les nodes lifecycle
_STARTUP_DELAY_S = 2.0
# Timeout pour chaque appel de service lifecycle
_SERVICE_TIMEOUT_S = 10.0
_TRANSITION_TIMEOUT_S = 5.0


class ModeManagerNode(Node):

    def __init__(self):
        super().__init__('mode_manager_node')

        self._current_mode: str | None = None
        self._node_active: dict[str, bool] = {n: False for n in _MANAGED_NODES}

        # Clients pour les services lifecycle de chaque node géré
        self._lc_clients: dict[str, ChangeState.Client] = {
            name: self.create_client(ChangeState, f'/{name}/change_state')
            for name in _MANAGED_NODES
        }

        # Queue de modes + thread worker unique : garantit que le DERNIER mode
        # demandé est toujours celui appliqué, sans race condition.
        self._mode_queue: queue.Queue[str] = queue.Queue()
        self._transition_lock = threading.Lock()
        self._worker = threading.Thread(target=self._mode_worker, daemon=True)
        self._worker.start()

        self.create_subscription(String, '/rover/mode', self._mode_cb, 10)

        self.get_logger().info('mode_manager_node démarré — initialisation lifecycle...')
        threading.Thread(target=self._startup_configure, daemon=True).start()

    # ── Initialisation au démarrage ───────────────────────────────────────────

    def _startup_configure(self):
        """
        Configure tous les nodes lifecycle au démarrage (Unconfigured → Inactive).
        Tourne dans un thread séparé pour ne pas bloquer le spin principal.
        Le thread principal (spin) traite les réponses de service pendant que
        ce thread attend les futures.
        """
        time.sleep(_STARTUP_DELAY_S)

        all_ok = True
        for name in _MANAGED_NODES:
            client = self._lc_clients[name]

            if not client.wait_for_service(timeout_sec=_SERVICE_TIMEOUT_S):
                self.get_logger().error(
                    f'[{name}] service change_state introuvable après {_SERVICE_TIMEOUT_S}s'
                )
                all_ok = False
                continue

            if self._transition_sync(name, Transition.TRANSITION_CONFIGURE):
                self.get_logger().info(f'[{name}] configuré (Inactive)')
            else:
                self.get_logger().error(f'[{name}] échec de la configuration')
                all_ok = False

        if all_ok:
            self.get_logger().info('Tous les nodes configurés — en attente du mode...')
        else:
            self.get_logger().warn('Initialisation incomplète — certains nodes non disponibles')

    # ── Réception du mode ─────────────────────────────────────────────────────

    def _mode_cb(self, msg: String):
        mode = msg.data

        if mode not in _MODE_MAP:
            self.get_logger().warn(f'Mode inconnu : "{mode}" — ignoré')
            return

        if mode == self._current_mode:
            return

        # On pousse dans la queue — le worker prend le dernier mode demandé
        self._mode_queue.put(mode)

    # ── Worker thread unique ──────────────────────────────────────────────────

    def _mode_worker(self):
        """
        Thread unique qui consomme la queue de modes.
        Si plusieurs modes sont en attente, seul le dernier est appliqué —
        élimine toute race condition entre changements rapides de mode.
        """
        while True:
            mode = self._mode_queue.get()
            # Vider la queue et garder uniquement le mode le plus récent
            while not self._mode_queue.empty():
                mode = self._mode_queue.get()
            self._apply_mode(mode)

    # ── Application du mode ───────────────────────────────────────────────────

    def _apply_mode(self, mode: str):
        """
        Applique les transitions lifecycle correspondant au nouveau mode.
        Le lock garantit qu'un seul changement de mode se traite à la fois.
        Ordre : désactivations d'abord (les nodes publient leurs zeros),
                activations ensuite.
        """
        with self._transition_lock:
            self.get_logger().info(f'Changement de mode : {self._current_mode!r} → {mode!r}')
            target = _MODE_MAP[mode]

            # 1. Désactiver ce qui ne doit pas tourner
            for name, should_be_active in target.items():
                if not should_be_active and self._node_active[name]:
                    if self._transition_sync(name, Transition.TRANSITION_DEACTIVATE):
                        self._node_active[name] = False
                        self.get_logger().info(f'[{name}] désactivé')
                    else:
                        self.get_logger().error(f'[{name}] échec désactivation')

            # 2. Activer ce qui doit tourner
            for name, should_be_active in target.items():
                if should_be_active and not self._node_active[name]:
                    if self._transition_sync(name, Transition.TRANSITION_ACTIVATE):
                        self._node_active[name] = True
                        self.get_logger().info(f'[{name}] activé')
                    else:
                        self.get_logger().error(f'[{name}] échec activation')

            self._current_mode = mode
            self.get_logger().info(f'Mode appliqué : {mode}')

    # ── Appel service lifecycle synchrone ─────────────────────────────────────

    def _transition_sync(self, node_name: str, transition_id: int) -> bool:
        """
        Envoie une requête de transition lifecycle et attend la réponse.
        Appelé depuis un thread worker — le thread principal (spin) traite
        la réponse DDS et résout la future pendant que ce thread poll.
        """
        client = self._lc_clients[node_name]
        req = ChangeState.Request()
        req.transition.id = transition_id
        future = client.call_async(req)

        deadline = time.monotonic() + _TRANSITION_TIMEOUT_S
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.02)

        if not future.done():
            self.get_logger().error(
                f'[{node_name}] timeout transition id={transition_id}'
            )
            return False

        result = future.result()
        return result is not None and result.success


def main(args=None):
    rclpy.init(args=args)
    node = ModeManagerNode()
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
