#!/usr/bin/env python3
"""
autonomous_node.py — Rover XPlore — Navigation Autonome (Mission 1)

Machine à états : SEARCH → AVOID → SCAN_ROTATE → CONFIRM_ARUCO → RETURN → DONE

Référence : autonomous_node_design_v1.1.docx
Testable sans hardware : ros2 topic pub pour simuler /distances, /aruco_detected, /ir_ground

Usage:
    ros2 run rover_commands autonomous_node
"""

import math
from enum import Enum, auto

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Float32MultiArray


# ══════════════════════════════════════════════════════════════════════
# ÉTATS DE LA MACHINE
# ══════════════════════════════════════════════════════════════════════

class State(Enum):
    SEARCH = auto()
    AVOID = auto()
    SCAN_ROTATE = auto()
    CONFIRM_ARUCO = auto()
    RETURN = auto()
    DONE = auto()


# ══════════════════════════════════════════════════════════════════════
# MESSAGE CUSTOM ARUCO (en attendant un vrai msg ROS2 custom)
# On utilise Float32MultiArray : [found(0/1), id, center_x, center_y, area]
# TODO: Remplacer par un message custom ArUcoDetection.msg quand le
#       package de messages sera créé
# ══════════════════════════════════════════════════════════════════════


class AutonomousNode(Node):
    """
    Nœud principal de navigation autonome.

    Subscribers :
        /distances        — Float32MultiArray [fl, fc, fr, l, r] en cm
        /aruco_detected   — Float32MultiArray [found, id, cx, cy, area]
        /ir_ground        — Float32MultiArray [ir_left, ir_right] (0=sol, 1=scotch)
        /rover_mode       — String ("AUTO" / "RACE" / "ARM")

    Publishers :
        /cmd_vel          — Twist (commande moteurs)
        /rover_status     — String (état courant, infos mission)
    """

    # ──────────────────────────────────────────────────────────────────
    # CONSTANTES — À CALIBRER AVEC LE VRAI HARDWARE
    # ──────────────────────────────────────────────────────────────────

    # Fréquence du timer principal (Hz)
    TICK_RATE = 10.0  # TODO: ajuster — 10-20 Hz selon charge CPU du RPi4

    # Ultrasons — seuils en cm
    US_OBSTACLE_THRESHOLD = 30.0       # TODO: calibrer 25-30 cm
    US_OBSTACLE_SIDE_THRESHOLD = 20.0  # TODO: calibrer pour capteurs latéraux

    # Pattern S
    SEARCH_LINEAR_SPEED = 0.25   # TODO: calibrer 0.2-0.3 m/s
    SEARCH_ANGULAR_SPEED = 0.8   # TODO: calibrer 0.5-1.0 rad/s pour virages 90°
    ROW_WIDTH_MIN = 0.40         # mètres — distance min entre rangées
    ROW_DISTANCE_MAX = 3.0       # mètres — force un virage si pas d'obstacle

    # AVOID
    AVOID_BACKUP_DISTANCE = 0.30  # mètres — recul en cas de cul-de-sac
    AVOID_BACKUP_SPEED = -0.15    # m/s (négatif = recul)
    AVOID_TURN_SPEED = 0.8        # rad/s pour pivot 90°
    AVOID_MAX_ROTATIONS = 3       # rebond infini → dégagement forcé

    # SCAN_ROTATE
    BORDER_COUNT_TRIGGER = 3      # rangées avant rotation 360°
    SCAN_ROTATE_SPEED = 0.3       # rad/s — lent pour éviter flou caméra

    # CONFIRM_ARUCO
    CONFIRM_N_FRAMES = 7          # TODO: calibrer 5-10 frames
    LOST_M_FRAMES = 15            # TODO: calibrer 15-20 frames
    APPROACH_DEADZONE = 0.10      # ±10% largeur image (zone morte guidage V1)
    APPROACH_LINEAR_SPEED = 0.15  # m/s — avance lente pendant approche
    APPROACH_ANGULAR_GAIN = 1.0   # TODO: calibrer — gain P pour guidage V1

    # RETURN
    RETURN_LINEAR_SPEED = 0.20    # m/s
    RETURN_ANGULAR_GAIN = 1.5     # TODO: calibrer — gain P pour cap vers (0,0)
    RETURN_BASE_SCAN_RADIUS = 0.50  # mètres — active scan ArUco base
    RETURN_STOP_RADIUS = 0.05     # mètres — arrêt fallback si pas de marker

    # Largeur image caméra (pixels) — pour le guidage V1
    IMAGE_WIDTH = 640  # TODO: confirmer avec la vraie caméra

    # ──────────────────────────────────────────────────────────────────
    # INIT
    # ──────────────────────────────────────────────────────────────────

    def __init__(self):
        super().__init__('autonomous_node')
        self.get_logger().info('autonomous_node démarré — état initial: SEARCH')

        # ── État machine ──
        self.state = State.SEARCH
        self.active = False  # activé seulement quand /rover_mode == "AUTO"

        # ── Données capteurs (mises à jour par les subscribers) ──
        self.distances = {
            'fl': 999.0,  # avant-gauche 45°
            'fc': 999.0,  # avant-centre 0°
            'fr': 999.0,  # avant-droit 45°
            'l':  999.0,  # gauche 90°
            'r':  999.0,  # droit 90°
        }
        self.ir_left = False   # True = scotch/bord détecté
        self.ir_right = False
        self.aruco = {
            'detected': False,
            'id': -1,
            'cx': 0.0,    # centre X dans l'image (pixels)
            'cy': 0.0,    # centre Y dans l'image (pixels)
            'area': 0.0,  # aire en pixels²
        }

        # ── Odométrie ──
        self.x = 0.0            # position X (mètres), origine = départ
        self.y = 0.0            # position Y (mètres)
        self.theta = 0.0        # cap (radians), 0 = direction initiale
        self.last_linear = 0.0  # dernière commande envoyée (pour intégration)
        self.last_angular = 0.0

        # ── Variables SEARCH ──
        self.heading_direction = 1       # +1 ou -1 (sens de la rangée)
        self.row_distance_traveled = 0.0 # distance parcourue sur la rangée courante

        # ── Variables AVOID ──
        self.avoid_return_state = State.SEARCH  # état à reprendre après AVOID
        self.avoid_rotation_count = 0           # compteur anti-rebond infini
        self.avoid_phase = 'EVALUATE'
        # TODO: implémenter les sous-états d'AVOID pour les manœuvres multi-étapes
        #       (reculer PUIS pivoter nécessite plusieurs cycles de tick)
        #       Phases possibles : EVALUATE → BACKING → TURNING → CLEAR

        # ── Variables SCAN_ROTATE ──
        self.border_count = 0            # rangées consécutives (bords map uniquement)
        self.scan_start_theta = 0.0      # cap au début de la rotation
        self.last_search_heading = 0.0   # direction mémorisée avant SCAN_ROTATE

        # ── Variables CONFIRM_ARUCO ──
        self.confirm_count = 0       # frames consécutives avec le bon ID
        self.lost_count = 0          # frames sans détection
        self.confirmed_id = -1       # ID en cours de confirmation
        self.approach_active = False  # sous-état APPROACH actif

        # ── Variables RETURN ──
        self.base_aruco_scan = False  # True quand on cherche le marker de base

        # ──────────────────────────────────────────────────────────────
        # SUBSCRIBERS
        # ──────────────────────────────────────────────────────────────

        self.sub_distances = self.create_subscription(
            Float32MultiArray, '/distances', self._cb_distances, 10
        )
        self.sub_aruco = self.create_subscription(
            Float32MultiArray, '/aruco_detected', self._cb_aruco, 10
        )
        self.sub_ir = self.create_subscription(
            Float32MultiArray, '/ir_ground', self._cb_ir_ground, 10
        )
        self.sub_mode = self.create_subscription(
            String, '/rover_mode', self._cb_rover_mode, 10
        )

        # ──────────────────────────────────────────────────────────────
        # PUBLISHERS
        # ──────────────────────────────────────────────────────────────

        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_status = self.create_publisher(String, '/rover_status', 10)

        # ──────────────────────────────────────────────────────────────
        # TIMER PRINCIPAL — cœur de la machine à états
        # ──────────────────────────────────────────────────────────────

        self.dt = 1.0 / self.TICK_RATE
        self.timer = self.create_timer(self.dt, self.tick)

    # ══════════════════════════════════════════════════════════════════
    # CALLBACKS SUBSCRIBERS
    # ══════════════════════════════════════════════════════════════════

    def _cb_distances(self, msg: Float32MultiArray):
        """Reçoit [fl, fc, fr, l, r] en cm depuis ultrasonic_node."""
        if len(msg.data) >= 5:
            self.distances['fl'] = msg.data[0]
            self.distances['fc'] = msg.data[1]
            self.distances['fr'] = msg.data[2]
            self.distances['l']  = msg.data[3]
            self.distances['r']  = msg.data[4]

    def _cb_aruco(self, msg: Float32MultiArray):
        """Reçoit [found(0/1), id, center_x, center_y, area] depuis aruco_detector_node."""
        if len(msg.data) >= 5:
            self.aruco['detected'] = msg.data[0] > 0.5
            self.aruco['id']   = int(msg.data[1])
            self.aruco['cx']   = msg.data[2]
            self.aruco['cy']   = msg.data[3]
            self.aruco['area'] = msg.data[4]

    def _cb_ir_ground(self, msg: Float32MultiArray):
        """
        Reçoit [ir_left, ir_right] depuis les capteurs IR au sol (TCRT5000).
        0 = sol normal, 1 = scotch/bord détecté.
        """
        if len(msg.data) >= 2:
            self.ir_left  = msg.data[0] > 0.5
            self.ir_right = msg.data[1] > 0.5

    def _cb_rover_mode(self, msg: String):
        """Active/désactive le nœud. Seul le mode 'AUTO' active autonomous_node."""
        was_active = self.active
        self.active = (msg.data == 'AUTO')

        if self.active and not was_active:
            self.get_logger().info('Mode AUTO activé — démarrage navigation')
            self._reset_mission()
        elif not self.active and was_active:
            self.get_logger().info('Mode AUTO désactivé — arrêt')
            self.publish_cmd(0.0, 0.0)

    # ══════════════════════════════════════════════════════════════════
    # TICK — BOUCLE PRINCIPALE
    # ══════════════════════════════════════════════════════════════════

    def tick(self):
        """
        Appelé à TICK_RATE Hz.
        1) Met à jour l'odométrie (toujours)
        2) Publie le statut
        3) Dispatch vers le handler de l'état courant
        """
        if not self.active:
            return

        self._update_odometry()
        self._publish_status()

        if self.state == State.SEARCH:
            self._run_search()
        elif self.state == State.AVOID:
            self._run_avoid()
        elif self.state == State.SCAN_ROTATE:
            self._run_scan_rotate()
        elif self.state == State.CONFIRM_ARUCO:
            self._run_confirm_aruco()
        elif self.state == State.RETURN:
            self._run_return()
        elif self.state == State.DONE:
            self._run_done()

    # ══════════════════════════════════════════════════════════════════
    # ODOMÉTRIE
    # ══════════════════════════════════════════════════════════════════

    def _update_odometry(self):
        """
        Intégration dead-reckoning basée sur les dernières commandes envoyées.
        Met à jour (x, y, theta) à chaque tick.

        NOTE: C'est une approximation. Avec des encodeurs moteur ou un IMU
        (MPU6050), on remplacerait par des données réelles.
        TODO: Remplacer par des données encodeurs/IMU si disponibles.
        """
        dx = self.last_linear * self.dt * math.cos(self.theta)
        dy = self.last_linear * self.dt * math.sin(self.theta)
        dtheta = self.last_angular * self.dt

        self.x += dx
        self.y += dy
        self.theta = self._normalize_angle(self.theta + dtheta)

        # Distance parcourue sur la rangée courante (pour sécurité distance max)
        self.row_distance_traveled += abs(self.last_linear) * self.dt

    # ══════════════════════════════════════════════════════════════════
    # ÉTAT : SEARCH — Balayage en Pattern S
    # ══════════════════════════════════════════════════════════════════

    def _run_search(self):
        """
        Avance en ligne droite, scanne pour ArUco en continu.
        Change de rangée au bord de map ou si distance max atteinte.
        """
        # ── Priorité 1 : ArUco détecté → CONFIRM_ARUCO ──
        if self.aruco['detected']:
            self.get_logger().info(
                f"ArUco ID={self.aruco['id']} détecté — passage en CONFIRM_ARUCO"
            )
            self._set_state(State.CONFIRM_ARUCO)
            return

        # ── Priorité 2 : Obstacle devant → AVOID ──
        if self._obstacle_front():
            self.get_logger().info('Obstacle devant — passage en AVOID')
            self.avoid_return_state = State.SEARCH
            self._set_state(State.AVOID)
            return

        # ── Priorité 3 : Bord de map (IR sol) → changement de rangée ──
        if self._border_detected():
            self.get_logger().info('Bord de map détecté — changement de rangée')
            self.border_count += 1  # seuls les bords comptent, PAS les obstacles
            self.row_distance_traveled = 0.0

            # Vérifier si on déclenche SCAN_ROTATE
            if self.border_count >= self.BORDER_COUNT_TRIGGER:
                self.get_logger().info(
                    f'{self.border_count} bords sans ArUco — SCAN_ROTATE'
                )
                self._set_state(State.SCAN_ROTATE)
                return

            self._do_row_change()
            return

        # ── Priorité 4 : Distance max → virage forcé ──
        if self.row_distance_traveled >= self.ROW_DISTANCE_MAX:
            self.get_logger().info('Distance max rangée — virage forcé')
            self._do_row_change()
            return

        # ── Comportement par défaut : avancer tout droit ──
        self.publish_cmd(self.SEARCH_LINEAR_SPEED, 0.0)

    def _do_row_change(self):
        """
        Changement de rangée du pattern S :
        Tourner 90° → avancer ROW_WIDTH_MIN → tourner 90° → repartir en sens inverse.

        TODO: IMPORTANT — cette méthode est un PLACEHOLDER.
              En vrai il faut un sous-état avec des phases car les virages
              prennent plusieurs ticks :
              
              ROW_CHANGE_TURN1  → tourne 90° (vérifie theta)
              ROW_CHANGE_SHIFT  → avance ROW_WIDTH_MIN (vérifie distance)
              ROW_CHANGE_TURN2  → tourne 90° en sens inverse
              ROW_CHANGE_DONE   → reprend SEARCH en ligne droite
              
              Pour l'instant, on se contente d'inverser la direction.
              Le comportement réel sera implémenté quand on testera sur hardware.
        """
        self.heading_direction *= -1
        self.row_distance_traveled = 0.0
        self.get_logger().info(
            f'Changement de rangée — direction: '
            f'{"→" if self.heading_direction > 0 else "←"}'
        )

    # ══════════════════════════════════════════════════════════════════
    # ÉTAT : AVOID — Évitement d'obstacles
    # ══════════════════════════════════════════════════════════════════

    def _run_avoid(self):
        """
        Évalue la situation avec les 5 ultrasons et choisit la manœuvre.

        Cas gérés (design doc v1.1 section 2.1) :
        - Obstacle devant seul         → tourne vers côté le plus libre
        - Obstacle devant + un côté    → tourne vers côté libre
        - Obstacle devant + deux côtés → recule + pivot 90° côté le - obstrué
        - Obstacle partout             → recule + pivot 90° côté le - obstrué
        - Rebond infini (>3 rotations) → dégagement forcé

        NE compte PAS comme changement de rangée (border_count inchangé).
        L'odométrie continue de s'accumuler.

        TODO: Implémenter les sous-phases pour les manœuvres multi-étapes.
              Actuellement chaque tick applique une commande instantanée,
              ce qui fonctionne pour tourner mais pas pour "reculer 30cm PUIS pivoter".
              Solution : sous-états BACKING (avec compteur distance) → TURNING.
        """
        fc = self.distances['fc']
        l  = self.distances['l']
        r  = self.distances['r']

        front_blocked = fc < self.US_OBSTACLE_THRESHOLD
        left_blocked  = l < self.US_OBSTACLE_SIDE_THRESHOLD
        right_blocked = r < self.US_OBSTACLE_SIDE_THRESHOLD

        # ── Voie libre devant → retour à l'état précédent ──
        if not front_blocked:
            self.get_logger().info(f'Voie libre — retour à {self.avoid_return_state.name}')
            self.avoid_rotation_count = 0
            self._set_state(self.avoid_return_state)
            return

        # ── Anti-rebond infini ──
        if self.avoid_rotation_count >= self.AVOID_MAX_ROTATIONS:
            self.get_logger().warn('Rebond infini — dégagement forcé')
            turn_dir = self.AVOID_TURN_SPEED if l >= r else -self.AVOID_TURN_SPEED
            self.publish_cmd(self.AVOID_BACKUP_SPEED, turn_dir)
            self.avoid_rotation_count = 0
            return

        # ── Obstacle devant + deux côtés → reculer + pivot ──
        if front_blocked and left_blocked and right_blocked:
            self.get_logger().info('Bloqué 3 côtés — recul + pivot')
            turn_dir = self.AVOID_TURN_SPEED if l >= r else -self.AVOID_TURN_SPEED
            self.publish_cmd(self.AVOID_BACKUP_SPEED, turn_dir)
            self.avoid_rotation_count += 1
            return

        # ── Obstacle devant + un côté → tourner vers le côté libre ──
        if front_blocked and left_blocked:
            self.publish_cmd(0.0, -self.AVOID_TURN_SPEED)  # tourne droite
            self.avoid_rotation_count += 1
            return

        if front_blocked and right_blocked:
            self.publish_cmd(0.0, self.AVOID_TURN_SPEED)   # tourne gauche
            self.avoid_rotation_count += 1
            return

        # ── Obstacle devant seul → tourne vers côté le plus libre ──
        if front_blocked:
            if l >= r:
                self.publish_cmd(0.0, self.AVOID_TURN_SPEED)   # gauche plus libre
            else:
                self.publish_cmd(0.0, -self.AVOID_TURN_SPEED)  # droite plus libre
            self.avoid_rotation_count += 1
            return

    # ══════════════════════════════════════════════════════════════════
    # ÉTAT : SCAN_ROTATE — Rotation 360° de scan
    # ══════════════════════════════════════════════════════════════════

    def _run_scan_rotate(self):
        """
        Rotation lente sur place. Analyse chaque frame pour ArUco.
        Si 360° sans détection → mémorise direction, repart différemment.
        """
        # ── ArUco détecté → CONFIRM_ARUCO ──
        if self.aruco['detected']:
            self.get_logger().info('ArUco pendant SCAN_ROTATE → CONFIRM_ARUCO')
            self._set_state(State.CONFIRM_ARUCO)
            return

        # ── Calculer l'angle parcouru depuis le début ──
        delta = abs(self.theta - self.scan_start_theta)
        if delta > math.pi:
            delta = 2.0 * math.pi - delta
        rotation_deg = math.degrees(delta)

        # ── 360° atteint → retour SEARCH dans une nouvelle direction ──
        if rotation_deg >= 350.0:  # marge de 10° pour ne pas rater le seuil
            self.get_logger().info('Rotation 360° complète — nouvelle direction')
            self.last_search_heading = self.theta
            self.border_count = 0  # reset le compteur
            self._do_row_change()  # virage supplémentaire = nouvelle zone
            self._set_state(State.SEARCH)
            return

        # ── Continuer la rotation ──
        self.publish_cmd(0.0, self.SCAN_ROTATE_SPEED)

    # ══════════════════════════════════════════════════════════════════
    # ÉTAT : CONFIRM_ARUCO — Confirmation du marker
    # ══════════════════════════════════════════════════════════════════

    def _run_confirm_aruco(self):
        """
        Rover arrêté. Compte les frames consécutives avec le même ID.
        Si détection instable → sous-état APPROACH (guidage V1).
        """
        # ── Première frame → enregistrer l'ID ──
        if self.confirmed_id == -1 and self.aruco['detected']:
            self.confirmed_id = self.aruco['id']
            self.confirm_count = 1
            self.lost_count = 0
            self.get_logger().info(f'Début confirmation ArUco ID={self.confirmed_id}')
            self.publish_cmd(0.0, 0.0)  # arrêt immédiat
            return

        # ── Même ID détecté → incrémenter ──
        if self.aruco['detected'] and self.aruco['id'] == self.confirmed_id:
            self.confirm_count += 1
            self.lost_count = 0

            if self.confirm_count >= self.CONFIRM_N_FRAMES:
                self.get_logger().info(
                    f'ArUco ID={self.confirmed_id} CONFIRMÉ '
                    f'({self.confirm_count} frames) — RETURN'
                )
                self._set_state(State.RETURN)
                return

            # Rester immobile tant que la confirmation avance
            if not self.approach_active:
                self.publish_cmd(0.0, 0.0)
            return

        # ── Marker perdu ou ID différent ──
        self.confirm_count = 0
        self.lost_count += 1

        if self.lost_count > self.LOST_M_FRAMES:
            self.get_logger().info(
                f'ArUco perdu ({self.lost_count} frames) — retour SEARCH'
            )
            self._reset_confirm()
            self._set_state(State.SEARCH)
            return

        # ── Sous-état APPROACH : se rapprocher ──
        self.approach_active = True
        self._approach_marker()

    def _approach_marker(self):
        """
        Guidage V1 — asservissement visuel sur le centre X du marker.
        - Marker à gauche  → angular > 0 (tourne gauche)
        - Marker à droite  → angular < 0 (tourne droite)
        - Centré (±10%)    → avance tout droit
        """
        if not self.aruco['detected']:
            self.publish_cmd(0.0, 0.0)  # perdu → arrêt, on attend
            return

        image_center_x = self.IMAGE_WIDTH / 2.0
        error_x = (self.aruco['cx'] - image_center_x) / image_center_x  # [-1, +1]

        if abs(error_x) < self.APPROACH_DEADZONE:
            # Centré → avancer
            self.publish_cmd(self.APPROACH_LINEAR_SPEED, 0.0)
        else:
            # error_x > 0 → marker à droite → tourner droite (angular négatif)
            angular = -self.APPROACH_ANGULAR_GAIN * error_x
            self.publish_cmd(0.0, angular)

    # ══════════════════════════════════════════════════════════════════
    # ÉTAT : RETURN — Retour à la base
    # ══════════════════════════════════════════════════════════════════

    def _run_return(self):
        """
        Navigue vers (0, 0) par vecteur direct (pas replay de chemin).
        À ~50 cm : scan ArUco base pour correction finale.
        Les obstacles déclenchent AVOID (avec retour → RETURN).
        """
        # ── Obstacle → AVOID temporaire ──
        if self._obstacle_front():
            self.get_logger().info('Obstacle pendant RETURN — AVOID')
            self.avoid_return_state = State.RETURN
            self._set_state(State.AVOID)
            return

        # ── Bord de map → aussi AVOID (on ne veut pas sortir) ──
        if self._border_detected():
            self.get_logger().info('Bord de map pendant RETURN — AVOID')
            self.avoid_return_state = State.RETURN
            self._set_state(State.AVOID)
            return

        # ── Calculs navigation ──
        dist = math.sqrt(self.x ** 2 + self.y ** 2)
        angle_to_origin = math.atan2(-self.y, -self.x)
        angle_error = self._normalize_angle(angle_to_origin - self.theta)

        # ── Phase finale : ~50 cm de la base → scan ArUco ──
        if dist < self.RETURN_BASE_SCAN_RADIUS:
            if not self.base_aruco_scan:
                self.get_logger().info(
                    f'À {dist:.2f}m de la base — activation scan ArUco base'
                )
                self.base_aruco_scan = True

            # TODO: Définir l'ID du marker de base (ou accepter tout ArUco)
            if self.aruco['detected']:
                self.get_logger().info('ArUco base détecté — correction + DONE')
                # TODO: Guidage V1 vers le marker de base pour arrêt précis
                self._set_state(State.DONE)
                return

            # Fallback : très proche sans marker → DONE quand même
            if dist < self.RETURN_STOP_RADIUS:
                self.get_logger().info(
                    'Position (0,0) atteinte sans ArUco base — DONE (fallback)'
                )
                self._set_state(State.DONE)
                return

        # ── Navigation vers (0, 0) ──
        angular_cmd = self.RETURN_ANGULAR_GAIN * angle_error
        # Ralentir à l'approche
        linear_cmd = self.RETURN_LINEAR_SPEED * min(1.0, dist / 1.0)

        # Erreur d'angle trop grande → tourner sur place d'abord
        if abs(angle_error) > math.radians(30):
            self.publish_cmd(0.0, angular_cmd)
        else:
            self.publish_cmd(linear_cmd, angular_cmd)

    # ══════════════════════════════════════════════════════════════════
    # ÉTAT : DONE — Mission accomplie
    # ══════════════════════════════════════════════════════════════════

    def _run_done(self):
        """Terminal. Moteurs arrêtés, publie MISSION_DONE."""
        self.publish_cmd(0.0, 0.0)
        msg = String()
        msg.data = 'MISSION_DONE'
        self.pub_status.publish(msg)

    # ══════════════════════════════════════════════════════════════════
    # HELPERS
    # ══════════════════════════════════════════════════════════════════

    def _set_state(self, new_state: State):
        """Transition d'état avec logging et initialisation."""
        old = self.state
        self.state = new_state
        self.get_logger().info(f'TRANSITION: {old.name} → {new_state.name}')

        # Initialisation à l'entrée
        if new_state == State.SEARCH:
            self.row_distance_traveled = 0.0
            # border_count n'est PAS reset ici — uniquement après SCAN_ROTATE

        elif new_state == State.AVOID:
            self.avoid_phase = 'EVALUATE'

        elif new_state == State.SCAN_ROTATE:
            self.scan_start_theta = self.theta
            self.last_search_heading = self.theta

        elif new_state == State.CONFIRM_ARUCO:
            self.confirm_count = 0
            self.lost_count = 0
            self.confirmed_id = -1
            self.approach_active = False
            self.publish_cmd(0.0, 0.0)

        elif new_state == State.RETURN:
            self.base_aruco_scan = False
            dist = math.sqrt(self.x ** 2 + self.y ** 2)
            self.get_logger().info(
                f'RETURN — pos=({self.x:.2f}, {self.y:.2f}), '
                f'distance origine={dist:.2f}m'
            )

        elif new_state == State.DONE:
            self.publish_cmd(0.0, 0.0)

    def _reset_mission(self):
        """Remet tout à zéro pour une nouvelle mission (activation mode AUTO)."""
        self.state = State.SEARCH
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_linear = 0.0
        self.last_angular = 0.0
        self.heading_direction = 1
        self.row_distance_traveled = 0.0
        self.border_count = 0
        self.avoid_rotation_count = 0
        self._reset_confirm()
        self.base_aruco_scan = False
        self.get_logger().info('Mission réinitialisée — odométrie (0, 0)')

    def _reset_confirm(self):
        """Reset les variables CONFIRM_ARUCO."""
        self.confirm_count = 0
        self.lost_count = 0
        self.confirmed_id = -1
        self.approach_active = False

    def _obstacle_front(self) -> bool:
        """True si un des 3 capteurs avant détecte un obstacle."""
        return (
            self.distances['fc'] < self.US_OBSTACLE_THRESHOLD
            or self.distances['fl'] < self.US_OBSTACLE_THRESHOLD
            or self.distances['fr'] < self.US_OBSTACLE_THRESHOLD
        )

    def _border_detected(self) -> bool:
        """True si un capteur IR détecte le scotch/bord de map."""
        return self.ir_left or self.ir_right

    def _publish_status(self):
        """Publie l'état courant sur /rover_status (pour status_monitor_node)."""
        msg = String()
        dist = math.sqrt(self.x ** 2 + self.y ** 2)
        msg.data = (
            f'state={self.state.name} '
            f'pos=({self.x:.2f},{self.y:.2f}) '
            f'theta={math.degrees(self.theta):.1f}deg '
            f'dist_origin={dist:.2f}m '
            f'borders={self.border_count}'
        )
        self.pub_status.publish(msg)

    def publish_cmd(self, linear: float, angular: float):
        """Publie Twist sur /cmd_vel et mémorise pour l'odométrie."""
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        self.pub_cmd_vel.publish(msg)
        self.last_linear = linear
        self.last_angular = angular

    @staticmethod
    def _normalize_angle(angle: float) -> float:
        """Normalise un angle dans [-pi, pi]."""
        return math.atan2(math.sin(angle), math.cos(angle))


# ══════════════════════════════════════════════════════════════════════
# MAIN
# ══════════════════════════════════════════════════════════════════════

def main(args=None):
    rclpy.init(args=args)
    node = AutonomousNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Arrêt par Ctrl+C')
    finally:
        node.publish_cmd(0.0, 0.0)  # sécurité: arrêt moteurs
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()