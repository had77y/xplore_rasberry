#!/usr/bin/env python3
# ══════════════════════════════════════════════════════════════════════════════
# autonomous_node.py — Raspberry Pi (package rover_commands)
#
# RÔLE : gérer la navigation autonome du rover via une machine à états (FSM).
#         Le rover explore la map, cherche le marker ArUco posé près de la cible,
#         s'en approche, puis revient à son point de départ.
#
# ATTENTION : ce fichier est un BROUILLON (version ancienne).
#   Le plan à jour est dans auto_plan.md (FSM : IDLE → LOCATE → APPROACH → RETURN).
#   Ce fichier implémente une FSM différente basée sur un balayage "pattern S".
#
# PLACE DANS LE SYSTÈME :
#
#   [RPi] aruco_node      → publie /aruco_detected [found, id, cx, cy, area]
#   [RPi] ultrasonic_node → publie /distances      [fl, fc, fr, l, r] cm
#   [RPi] ir_ground_node  → publie /ir_ground      [ir_left, ir_right]
#   [PC]  controller_node → publie /rover_mode     "AUTO" / "RACE" / "ARM"
#          ↓ tous ces topics
#   [RPi] autonomous_node ← CE FICHIER
#          ↓ publie
#   /cmd_vel          (geometry_msgs/Twist) → motor_controller_node → Arduino → roues
#   /rover_status     (std_msgs/String)     → PC (information sur l'état)
#
# MACHINE À ÉTATS :
#
#   SEARCH ──── ArUco vu ──────────────────────────────→ CONFIRM_ARUCO
#     ↑  ↑ ───── obstacle ──────────────────────────────→ AVOID ──→ (retour à SEARCH)
#     ↑  └─ bord map (IR) → changement de rangée
#     ↑  └─ 3 bords sans ArUco ──────────────────────→ SCAN_ROTATE
#     ↑                                                     ↓ 360° sans ArUco
#     ←─────────────────────────────────────────────────────┘
#
#   CONFIRM_ARUCO ── 7 frames consécutives avec même ID ──→ RETURN
#     ↑  └─ marker perdu >15 frames ──────────────────────→ SEARCH
#     └─ marker partiellement visible → APPROACH (asservissement visuel)
#
#   RETURN ── atteint (0,0) et ArUco base détecté ────────→ DONE
#     └─ obstacle pendant retour ──────────────────────────→ AVOID ──→ (retour à RETURN)
#
#   DONE : terminal — moteurs arrêtés, publie MISSION_DONE
#
# ODOMÉTRIE :
#   Dead-reckoning simple : intégration des commandes envoyées (pas d'encodeurs réels).
#   x, y, theta sont estimés à partir des vitesses commandées × temps.
#   Imprécis sur longue distance (dérive), mais suffisant pour retrouver (0,0).
# ══════════════════════════════════════════════════════════════════════════════

import math
from enum import Enum, auto

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Float32MultiArray


# ── États de la machine ───────────────────────────────────────────────────────
# auto() génère automatiquement des valeurs entières uniques pour chaque état
class State(Enum):
    SEARCH        = auto()  # balayage en pattern S, cherche l'ArUco
    AVOID         = auto()  # obstacle détecté → manœuvre d'évitement
    SCAN_ROTATE   = auto()  # rotation 360° sur place pour scanner l'environnement
    CONFIRM_ARUCO = auto()  # ArUco vu → confirmation sur N frames + approche visuelle
    RETURN        = auto()  # ArUco confirmé → retour vers (0,0)
    DONE          = auto()  # mission terminée


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

    # ── Constantes de navigation — À CALIBRER sur le hardware réel ───────────

    TICK_RATE = 10.0  # Hz — fréquence de la boucle principale (10 fois par seconde)

    # Seuils ultrasoniques (cm) : en-dessous → obstacle détecté
    US_OBSTACLE_THRESHOLD      = 30.0  # capteurs avant (fl, fc, fr)
    US_OBSTACLE_SIDE_THRESHOLD = 20.0  # capteurs latéraux (l, r)

    # SEARCH — balayage en lignes droites
    SEARCH_LINEAR_SPEED  = 0.25  # m/s — vitesse d'avancement en ligne droite
    SEARCH_ANGULAR_SPEED = 0.8   # rad/s — vitesse de rotation pour les virages
    ROW_WIDTH_MIN        = 0.40  # m — largeur minimale entre deux rangées de balayage
    ROW_DISTANCE_MAX     = 3.0   # m — force un virage si aucun obstacle/bord n'est trouvé

    # AVOID — manœuvres d'évitement d'obstacle
    AVOID_BACKUP_DISTANCE = 0.30  # m — distance de recul
    AVOID_BACKUP_SPEED    = -0.15 # m/s négatif = reculer
    AVOID_TURN_SPEED      = 0.8   # rad/s — vitesse de rotation pour dégagement
    AVOID_MAX_ROTATIONS   = 3     # max de rotations avant dégagement forcé (anti-boucle infinie)

    # SCAN_ROTATE — rotation 360° de scan
    BORDER_COUNT_TRIGGER = 3    # nombre de bords consécutifs sans ArUco avant de faire un 360°
    SCAN_ROTATE_SPEED    = 0.3  # rad/s — lent pour ne pas flouter l'image caméra

    # CONFIRM_ARUCO — confirmation et approche visuelle
    CONFIRM_N_FRAMES    = 7    # frames consécutives avec le même ID pour valider l'ArUco
    LOST_M_FRAMES       = 15   # frames sans détection avant de revenir à SEARCH
    APPROACH_DEADZONE   = 0.10 # ±10% de la largeur image = zone morte (pas de correction)
    APPROACH_LINEAR_SPEED  = 0.15  # m/s — vitesse d'approche lente
    APPROACH_ANGULAR_GAIN  = 1.0   # gain P du contrôleur proportionnel pour centrer le marker

    # RETURN — retour à la base
    RETURN_LINEAR_SPEED    = 0.20  # m/s
    RETURN_ANGULAR_GAIN    = 1.5   # gain P pour s'orienter vers (0,0)
    RETURN_BASE_SCAN_RADIUS = 0.50 # m — à cette distance de la base, cherche l'ArUco de base
    RETURN_STOP_RADIUS      = 0.05 # m — arrêt de secours si pas de marker de base

    # Largeur image caméra (pixels) — pour le calcul de l'erreur de centrage
    IMAGE_WIDTH = 640

    def __init__(self):
        super().__init__('autonomous_node')
        self.get_logger().info('autonomous_node démarré — état initial: SEARCH')

        # ── Machine à états ───────────────────────────────────────────────────
        self.state  = State.SEARCH
        self.active = False  # le node n'agit que quand /rover_mode == "AUTO"

        # ── Données capteurs (mises à jour par les callbacks) ─────────────────
        # Initialisées à 999 = "pas d'obstacle" (valeur sûre par défaut)
        self.distances = {
            'fl': 999.0,  # avant-gauche 45°
            'fc': 999.0,  # avant-centre  0°
            'fr': 999.0,  # avant-droite 45°
            'l':  999.0,  # gauche 90°
            'r':  999.0,  # droite 90°
        }
        self.ir_left  = False  # True = scotch/bord détecté côté gauche
        self.ir_right = False  # True = scotch/bord détecté côté droit
        self.aruco = {
            'detected': False,
            'id':    -1,
            'cx':   0.0,    # centre X dans l'image (0=gauche, 640=droite)
            'cy':   0.0,    # centre Y dans l'image (0=haut, 480=bas)
            'area': 0.0,    # aire en pixels² (plus grand = plus proche)
        }

        # ── Odométrie (dead-reckoning) ────────────────────────────────────────
        # On intègre les commandes envoyées pour estimer la position.
        # (0,0) = position de départ. theta = cap (0 = direction initiale).
        self.x           = 0.0
        self.y           = 0.0
        self.theta       = 0.0
        self.last_linear  = 0.0  # dernière vitesse linéaire commandée
        self.last_angular = 0.0  # dernière vitesse angulaire commandée

        # ── Variables SEARCH ──────────────────────────────────────────────────
        self.heading_direction    = 1    # +1 = direction courante, -1 = direction inverse
        self.row_distance_traveled = 0.0 # distance parcourue sur la rangée courante

        # ── Variables AVOID ───────────────────────────────────────────────────
        self.avoid_return_state  = State.SEARCH  # état à reprendre après l'évitement
        self.avoid_rotation_count = 0             # compte les rotations (anti-rebond)
        self.avoid_phase          = 'EVALUATE'

        # ── Variables SCAN_ROTATE ─────────────────────────────────────────────
        self.border_count        = 0    # rangées consécutives sans ArUco
        self.scan_start_theta    = 0.0  # cap au moment où on commence le 360°
        self.last_search_heading = 0.0  # cap mémorisé avant SCAN_ROTATE

        # ── Variables CONFIRM_ARUCO ───────────────────────────────────────────
        self.confirm_count   = 0     # frames avec le même ID consécutivement
        self.lost_count      = 0     # frames sans détection
        self.confirmed_id    = -1    # ID qu'on est en train de confirmer (-1 = aucun)
        self.approach_active = False # sous-état d'approche visuelle actif

        # ── Variables RETURN ──────────────────────────────────────────────────
        self.base_aruco_scan = False  # True quand on cherche le marker de base

        # ── Subscribers ───────────────────────────────────────────────────────
        # /distances : publié par ultrasonic_node (RPi) — 5 capteurs US en cm
        self.sub_distances = self.create_subscription(
            Float32MultiArray, '/distances', self._cb_distances, 10
        )
        # /aruco_detected : publié par aruco_node (RPi) — position du marker dans l'image
        self.sub_aruco = self.create_subscription(
            Float32MultiArray, '/aruco_detected', self._cb_aruco, 10
        )
        # /ir_ground : capteurs IR au sol — détecte le scotch de délimitation de la map
        self.sub_ir = self.create_subscription(
            Float32MultiArray, '/ir_ground', self._cb_ir_ground, 10
        )
        # /rover_mode : publié par controller_node (PC) — active/désactive ce node
        # Note : ce topic utilise "AUTO" majuscule (différent de /rover/mode qui utilise "autonomous")
        self.sub_mode = self.create_subscription(
            String, '/rover_mode', self._cb_rover_mode, 10
        )

        # ── Publishers ────────────────────────────────────────────────────────
        # /cmd_vel → reçu par motor_controller_node qui envoie à l'Arduino
        self.pub_cmd_vel = self.create_publisher(Twist,  '/cmd_vel',      10)
        # /rover_status → reçu par le PC pour afficher l'état dans la GUI
        self.pub_status  = self.create_publisher(String, '/rover_status',  10)

        # ── Timer principal — cœur de la machine à états ──────────────────────
        # Appelé à TICK_RATE Hz. C'est ici que toute la logique de navigation s'exécute.
        self.dt    = 1.0 / self.TICK_RATE
        self.timer = self.create_timer(self.dt, self.tick)

    # ── Callbacks des subscribers ─────────────────────────────────────────────

    def _cb_distances(self, msg: Float32MultiArray):
        """Reçoit [fl, fc, fr, l, r] en cm depuis ultrasonic_node."""
        if len(msg.data) >= 5:
            self.distances['fl'] = msg.data[0]
            self.distances['fc'] = msg.data[1]
            self.distances['fr'] = msg.data[2]
            self.distances['l']  = msg.data[3]
            self.distances['r']  = msg.data[4]

    def _cb_aruco(self, msg: Float32MultiArray):
        """
        Reçoit [found(0/1), id, center_x, center_y, area] depuis aruco_node.
        center_x : 0 = bord gauche de l'image, 640 = bord droit
        area     : plus grand = marker plus proche du rover
        """
        if len(msg.data) >= 5:
            self.aruco['detected'] = msg.data[0] > 0.5  # 1.0 → True, 0.0 → False
            self.aruco['id']       = int(msg.data[1])
            self.aruco['cx']       = msg.data[2]
            self.aruco['cy']       = msg.data[3]
            self.aruco['area']     = msg.data[4]

    def _cb_ir_ground(self, msg: Float32MultiArray):
        """
        Reçoit [ir_left, ir_right] depuis les capteurs IR sol (TCRT5000).
        0.0 = sol normal (sombre ou clair selon calibration)
        1.0 = scotch blanc détecté → bord de la map
        """
        if len(msg.data) >= 2:
            self.ir_left  = msg.data[0] > 0.5
            self.ir_right = msg.data[1] > 0.5

    def _cb_rover_mode(self, msg: String):
        """
        Reçoit le mode depuis /rover_mode (publié par controller_node PC).
        'AUTO' → active ce node et démarre la mission depuis zéro
        Autre → désactive, stoppe les moteurs
        """
        was_active  = self.active
        self.active = (msg.data == 'AUTO')

        if self.active and not was_active:
            self.get_logger().info('Mode AUTO activé — démarrage navigation')
            self._reset_mission()  # reset de l'odométrie et de tous les états
        elif not self.active and was_active:
            self.get_logger().info('Mode AUTO désactivé — arrêt')
            self.publish_cmd(0.0, 0.0)  # stoppe les moteurs immédiatement

    # ── Boucle principale ─────────────────────────────────────────────────────

    def tick(self):
        """
        Cœur de la machine à états — appelée à TICK_RATE Hz (10 fois/s).
        Ordre d'exécution :
          1. Mise à jour odométrie (toujours, même si non actif)
          2. Publication du statut pour la GUI PC
          3. Dispatch vers le handler de l'état courant
        """
        if not self.active:
            return  # node désactivé → ne rien faire

        self._update_odometry()  # met à jour x, y, theta
        self._publish_status()   # envoie l'état sur /rover_status

        # Dispatch selon l'état courant
        if   self.state == State.SEARCH:        self._run_search()
        elif self.state == State.AVOID:         self._run_avoid()
        elif self.state == State.SCAN_ROTATE:   self._run_scan_rotate()
        elif self.state == State.CONFIRM_ARUCO: self._run_confirm_aruco()
        elif self.state == State.RETURN:        self._run_return()
        elif self.state == State.DONE:          self._run_done()

    # ── Odométrie dead-reckoning ───────────────────────────────────────────────

    def _update_odometry(self):
        """
        Estimation de la position à partir des commandes envoyées (dead-reckoning).
        Principe : si on avance à 0.25 m/s pendant 0.1s → on a avancé de 0.025m
        dans la direction theta.

        Formules :
          dx     = linear × dt × cos(theta)
          dy     = linear × dt × sin(theta)
          dtheta = angular × dt

        Limitation : les erreurs s'accumulent (glissement roues, terrain).
        À remplacer par une vraie odométrie basée sur les encodeurs quand ils seront disponibles.
        """
        dx     = self.last_linear  * self.dt * math.cos(self.theta)
        dy     = self.last_linear  * self.dt * math.sin(self.theta)
        dtheta = self.last_angular * self.dt

        self.x     += dx
        self.y     += dy
        self.theta  = self._normalize_angle(self.theta + dtheta)

        # Aussi utilisé par SEARCH pour savoir quand forcer un virage
        self.row_distance_traveled += abs(self.last_linear) * self.dt

    # ── ÉTAT : SEARCH ─────────────────────────────────────────────────────────

    def _run_search(self):
        """
        Balayage en "pattern S" : le rover avance en lignes droites
        et change de rangée quand il atteint un bord ou une distance max.

        Priorités (vérifiées dans l'ordre) :
          1. ArUco détecté → CONFIRM_ARUCO (mission presque accomplie !)
          2. Obstacle devant → AVOID
          3. Bord de map (IR sol) → changement de rangée
          4. Distance max de rangée → virage forcé
          5. Par défaut → avancer tout droit
        """
        # Priorité 1 : ArUco détecté → transition immédiate
        if self.aruco['detected']:
            self.get_logger().info(
                f"ArUco ID={self.aruco['id']} détecté — passage en CONFIRM_ARUCO"
            )
            self._set_state(State.CONFIRM_ARUCO)
            return

        # Priorité 2 : obstacle devant → éviter avant de continuer
        if self._obstacle_front():
            self.get_logger().info('Obstacle devant — passage en AVOID')
            self.avoid_return_state = State.SEARCH  # on reviendra ici après l'évitement
            self._set_state(State.AVOID)
            return

        # Priorité 3 : bord de map (le rover a atteint le scotch délimitant la zone)
        if self._border_detected():
            self.get_logger().info('Bord de map détecté — changement de rangée')
            self.border_count += 1       # incrémente le compteur de bords
            self.row_distance_traveled = 0.0

            # Si on a atteint 3 bords sans trouver l'ArUco → rotation 360° de scan
            if self.border_count >= self.BORDER_COUNT_TRIGGER:
                self.get_logger().info(
                    f'{self.border_count} bords sans ArUco — SCAN_ROTATE'
                )
                self._set_state(State.SCAN_ROTATE)
                return

            self._do_row_change()  # demi-tour pour la rangée suivante
            return

        # Priorité 4 : distance max atteinte sans bord → virage forcé
        # (ex: obstacle latéral qui empêche de voir le bord)
        if self.row_distance_traveled >= self.ROW_DISTANCE_MAX:
            self.get_logger().info('Distance max rangée — virage forcé')
            self._do_row_change()
            return

        # Comportement par défaut : avancer en ligne droite, scanner la caméra
        self.publish_cmd(self.SEARCH_LINEAR_SPEED, 0.0)

    def _do_row_change(self):
        """
        Changement de rangée du pattern S : inverser la direction de balayage.
        TODO : implémenter vraiment les virages en sous-états (90° + décalage + 90°).
        Pour l'instant, on inverse juste la direction (placeholder).
        """
        self.heading_direction     *= -1  # inverse le sens de balayage
        self.row_distance_traveled  = 0.0

    # ── ÉTAT : AVOID ──────────────────────────────────────────────────────────

    def _run_avoid(self):
        """
        Évalue la situation avec les 5 capteurs ultrasoniques et manœuvre.

        Logique :
          - Aucun obstacle → retour à l'état précédent (SEARCH ou RETURN)
          - Obstacle devant seul → tourne vers le côté le plus libre
          - Obstacle devant + un côté → tourne vers le côté libre
          - Obstacle partout → recule + pivote (côté le moins bouché)
          - Trop de rotations (>3) → dégagement forcé (anti-blocage infini)

        NE compte PAS comme changement de rangée (border_count inchangé).
        """
        fc = self.distances['fc']
        l  = self.distances['l']
        r  = self.distances['r']

        front_blocked = fc < self.US_OBSTACLE_THRESHOLD
        left_blocked  = l  < self.US_OBSTACLE_SIDE_THRESHOLD
        right_blocked = r  < self.US_OBSTACLE_SIDE_THRESHOLD

        # Voie libre devant → la manœuvre est terminée, reprendre la mission
        if not front_blocked:
            self.get_logger().info(f'Voie libre — retour à {self.avoid_return_state.name}')
            self.avoid_rotation_count = 0  # reset le compteur d'anti-rebond
            self._set_state(self.avoid_return_state)
            return

        # Anti-rebond : si on a tourné >3 fois sans se dégager → dégagement forcé
        # (évite que le rover tourne en boucle face à un mur)
        if self.avoid_rotation_count >= self.AVOID_MAX_ROTATIONS:
            self.get_logger().warn('Rebond infini — dégagement forcé')
            # Recule ET tourne en même temps vers le côté le plus libre
            turn_dir = self.AVOID_TURN_SPEED if l >= r else -self.AVOID_TURN_SPEED
            self.publish_cmd(self.AVOID_BACKUP_SPEED, turn_dir)
            self.avoid_rotation_count = 0  # reset pour ne pas rester bloqué ici
            return

        # Bloqué des 3 côtés → recul + pivot vers le moins obstrué
        if front_blocked and left_blocked and right_blocked:
            self.get_logger().info('Bloqué 3 côtés — recul + pivot')
            turn_dir = self.AVOID_TURN_SPEED if l >= r else -self.AVOID_TURN_SPEED
            self.publish_cmd(self.AVOID_BACKUP_SPEED, turn_dir)
            self.avoid_rotation_count += 1
            return

        # Obstacle devant + gauche → tourner à droite (seule sortie)
        if front_blocked and left_blocked:
            self.publish_cmd(0.0, -self.AVOID_TURN_SPEED)
            self.avoid_rotation_count += 1
            return

        # Obstacle devant + droite → tourner à gauche (seule sortie)
        if front_blocked and right_blocked:
            self.publish_cmd(0.0, self.AVOID_TURN_SPEED)
            self.avoid_rotation_count += 1
            return

        # Obstacle devant seulement → tourner vers le côté le plus libre
        # l >= r : gauche plus libre → tourne à gauche (angular positif)
        if front_blocked:
            if l >= r:
                self.publish_cmd(0.0,  self.AVOID_TURN_SPEED)
            else:
                self.publish_cmd(0.0, -self.AVOID_TURN_SPEED)
            self.avoid_rotation_count += 1
            return

    # ── ÉTAT : SCAN_ROTATE ────────────────────────────────────────────────────

    def _run_scan_rotate(self):
        """
        Rotation lente sur place (360°) pour chercher l'ArUco dans toutes les directions.
        Déclenché après BORDER_COUNT_TRIGGER=3 changements de rangée sans trouver le marker.

        Logique :
          - Détecte l'ArUco pendant la rotation → CONFIRM_ARUCO immédiatement
          - 360° complets sans ArUco → repart en SEARCH dans une nouvelle direction
        """
        # ArUco vu pendant la rotation → ne pas finir le 360°, aller confirmer
        if self.aruco['detected']:
            self.get_logger().info('ArUco pendant SCAN_ROTATE → CONFIRM_ARUCO')
            self._set_state(State.CONFIRM_ARUCO)
            return

        # Calcul de l'angle parcouru depuis le début de la rotation
        # (différence entre theta actuel et theta au moment où on a commencé)
        delta = abs(self.theta - self.scan_start_theta)
        if delta > math.pi:
            # Correction pour le passage de -π à +π (discontinuité)
            delta = 2.0 * math.pi - delta
        rotation_deg = math.degrees(delta)

        # 360° atteint (avec marge de 10° pour ne pas rater le seuil à cause du dt)
        if rotation_deg >= 350.0:
            self.get_logger().info('Rotation 360° complète — nouvelle direction')
            self.last_search_heading = self.theta
            self.border_count = 0  # reset pour repartir vers les bords
            self._do_row_change()
            self._set_state(State.SEARCH)
            return

        # Continuer à tourner lentement
        self.publish_cmd(0.0, self.SCAN_ROTATE_SPEED)

    # ── ÉTAT : CONFIRM_ARUCO ──────────────────────────────────────────────────

    def _run_confirm_aruco(self):
        """
        Phase de confirmation : on veut être sûr que l'ArUco est bien le bon
        avant de déclencher le retour à la base.

        Logique :
          - On compte les frames consécutives avec le même ID
          - Si 7 frames consécutives avec le même ID → RETURN (c'est confirmé)
          - Si le marker disparaît > 15 frames → SEARCH (c'était peut-être un faux positif)
          - Si le marker est instable (perd quelques frames) → APPROACH
            (on avance doucement vers lui pour mieux le voir)
        """
        # Première frame : on mémorise l'ID et on s'arrête
        if self.confirmed_id == -1 and self.aruco['detected']:
            self.confirmed_id = self.aruco['id']
            self.confirm_count = 1
            self.lost_count    = 0
            self.get_logger().info(f'Début confirmation ArUco ID={self.confirmed_id}')
            self.publish_cmd(0.0, 0.0)  # arrêt pour stabiliser l'image
            return

        # Même ID détecté → compter
        if self.aruco['detected'] and self.aruco['id'] == self.confirmed_id:
            self.confirm_count += 1
            self.lost_count     = 0

            if self.confirm_count >= self.CONFIRM_N_FRAMES:
                # CONFIRM_N_FRAMES=7 frames consécutives → ArUco validé !
                self.get_logger().info(
                    f'ArUco ID={self.confirmed_id} CONFIRMÉ '
                    f'({self.confirm_count} frames) — RETURN'
                )
                self._set_state(State.RETURN)  # mission principale accomplie
                return

            # Confirmation en cours → rester immobile (sauf si approach actif)
            if not self.approach_active:
                self.publish_cmd(0.0, 0.0)
            return

        # Marker perdu ou ID différent
        self.confirm_count = 0
        self.lost_count   += 1

        if self.lost_count > self.LOST_M_FRAMES:
            # Trop longtemps sans détection → c'était probablement un faux positif
            self.get_logger().info(
                f'ArUco perdu ({self.lost_count} frames) — retour SEARCH'
            )
            self._reset_confirm()
            self._set_state(State.SEARCH)
            return

        # Marker partiellement visible → s'approcher pour mieux voir
        self.approach_active = True
        self._approach_marker()

    def _approach_marker(self):
        """
        Asservissement visuel simple (proportionnel) sur le centre X du marker.
        Objectif : centrer le marker dans l'image en tournant légèrement.

        error_x = (cx - 320) / 320
          → -1 = marker complètement à gauche de l'image
          → +1 = marker complètement à droite de l'image
          →  0 = marker centré

        Si |error_x| < DEADZONE=0.10 (±10%) → avancer tout droit
        Sinon → corriger en tournant (sans avancer)
        """
        if not self.aruco['detected']:
            self.publish_cmd(0.0, 0.0)  # perdu → s'arrêter et attendre
            return

        image_center_x = self.IMAGE_WIDTH / 2.0
        # Normalisation : error_x dans [-1, +1]
        error_x = (self.aruco['cx'] - image_center_x) / image_center_x

        if abs(error_x) < self.APPROACH_DEADZONE:
            # Marker centré → avancer lentement vers lui
            self.publish_cmd(self.APPROACH_LINEAR_SPEED, 0.0)
        else:
            # error_x > 0 → marker à droite → angular négatif = tourner à droite
            # error_x < 0 → marker à gauche → angular positif = tourner à gauche
            angular = -self.APPROACH_ANGULAR_GAIN * error_x
            self.publish_cmd(0.0, angular)

    # ── ÉTAT : RETURN ─────────────────────────────────────────────────────────

    def _run_return(self):
        """
        Retour à la position de départ (0, 0) par navigation directe.
        Utilise l'odométrie dead-reckoning pour estimer la direction vers (0,0).

        Logique :
          1. Obstacle ou bord → AVOID temporaire (puis retour ici)
          2. À ~50cm de la base → cherche le marker ArUco de base pour positionnement précis
          3. Navigation vers (0,0) : calcul de l'angle vers l'origine, correction P
          4. Si angle > 30° → tourner d'abord sur place, PUIS avancer
             (évite d'aller en crabe si on est très mal orienté)
        """
        # Obstacle → évitement temporaire, on reviendra en RETURN après
        if self._obstacle_front():
            self.get_logger().info('Obstacle pendant RETURN — AVOID')
            self.avoid_return_state = State.RETURN
            self._set_state(State.AVOID)
            return

        # Bord de map → aussi passer par AVOID (on ne veut pas sortir de la zone)
        if self._border_detected():
            self.get_logger().info('Bord de map pendant RETURN — AVOID')
            self.avoid_return_state = State.RETURN
            self._set_state(State.AVOID)
            return

        # Calculs géométriques vers l'origine (0,0)
        dist          = math.sqrt(self.x ** 2 + self.y ** 2)              # distance à l'origine
        angle_to_origin = math.atan2(-self.y, -self.x)                   # direction vers (0,0)
        angle_error   = self._normalize_angle(angle_to_origin - self.theta)  # erreur de cap

        # Phase finale : à ~50cm → cherche un marker ArUco de base pour affiner
        if dist < self.RETURN_BASE_SCAN_RADIUS:
            if not self.base_aruco_scan:
                self.get_logger().info(
                    f'À {dist:.2f}m de la base — activation scan ArUco base'
                )
                self.base_aruco_scan = True

            if self.aruco['detected']:
                # Marker de base détecté → DONE
                self.get_logger().info('ArUco base détecté — correction + DONE')
                self._set_state(State.DONE)
                return

            # Fallback : très proche sans marker → arrêt quand même
            if dist < self.RETURN_STOP_RADIUS:
                self.get_logger().info(
                    'Position (0,0) atteinte sans ArUco base — DONE (fallback)'
                )
                self._set_state(State.DONE)
                return

        # Navigation vers (0,0)
        angular_cmd = self.RETURN_ANGULAR_GAIN * angle_error
        # Ralentir à l'approche : linear = vitesse × min(1, dist/1m)
        # À 1m → vitesse pleine ; à 0.5m → demi-vitesse ; à 0m → 0
        linear_cmd  = self.RETURN_LINEAR_SPEED * min(1.0, dist / 1.0)

        # Si on est très mal orienté → tourner d'abord sur place (>30°)
        # Sinon → avancer avec correction angulaire simultanée
        if abs(angle_error) > math.radians(30):
            self.publish_cmd(0.0, angular_cmd)
        else:
            self.publish_cmd(linear_cmd, angular_cmd)

    # ── ÉTAT : DONE ───────────────────────────────────────────────────────────

    def _run_done(self):
        """
        État terminal. Le rover s'est arrêté à la base.
        Publie MISSION_DONE sur /rover_status (la GUI PC peut afficher un message).
        """
        self.publish_cmd(0.0, 0.0)
        msg = String()
        msg.data = 'MISSION_DONE'
        self.pub_status.publish(msg)

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _set_state(self, new_state: State):
        """
        Transition d'état avec logging et initialisation des variables de chaque état.
        Toujours utiliser cette méthode pour changer d'état (jamais self.state = X directement).
        """
        old = self.state
        self.state = new_state
        self.get_logger().info(f'TRANSITION: {old.name} → {new_state.name}')

        # Initialisation à l'entrée de chaque état
        if new_state == State.SEARCH:
            self.row_distance_traveled = 0.0
            # border_count n'est PAS reset ici, seulement après SCAN_ROTATE

        elif new_state == State.AVOID:
            self.avoid_phase = 'EVALUATE'

        elif new_state == State.SCAN_ROTATE:
            self.scan_start_theta    = self.theta  # mémorise le cap de début de rotation
            self.last_search_heading = self.theta

        elif new_state == State.CONFIRM_ARUCO:
            # Reset complet de la confirmation
            self.confirm_count   = 0
            self.lost_count      = 0
            self.confirmed_id    = -1
            self.approach_active = False
            self.publish_cmd(0.0, 0.0)  # s'arrêter pour stabiliser l'image

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
        """
        Remet tout à zéro pour une nouvelle mission (quand le mode AUTO est activé).
        L'odométrie repart de (0, 0, 0) = position de départ.
        """
        self.state                 = State.SEARCH
        self.x                     = 0.0
        self.y                     = 0.0
        self.theta                 = 0.0
        self.last_linear           = 0.0
        self.last_angular          = 0.0
        self.heading_direction     = 1
        self.row_distance_traveled = 0.0
        self.border_count          = 0
        self.avoid_rotation_count  = 0
        self._reset_confirm()
        self.base_aruco_scan       = False
        self.get_logger().info('Mission réinitialisée — odométrie (0, 0)')

    def _reset_confirm(self):
        """Reset les variables de CONFIRM_ARUCO."""
        self.confirm_count   = 0
        self.lost_count      = 0
        self.confirmed_id    = -1
        self.approach_active = False

    def _obstacle_front(self) -> bool:
        """
        True si AU MOINS UN des 3 capteurs avant détecte un obstacle.
        fl = avant-gauche 45°, fc = avant-centre, fr = avant-droite 45°
        Seuil = US_OBSTACLE_THRESHOLD = 30 cm
        """
        return (
            self.distances['fc'] < self.US_OBSTACLE_THRESHOLD
            or self.distances['fl'] < self.US_OBSTACLE_THRESHOLD
            or self.distances['fr'] < self.US_OBSTACLE_THRESHOLD
        )

    def _border_detected(self) -> bool:
        """
        True si l'un des capteurs IR sol détecte le scotch de délimitation de la map.
        Utilisé dans SEARCH (changement de rangée) et RETURN (évitement de sortie).
        """
        return self.ir_left or self.ir_right

    def _publish_status(self):
        """
        Publie l'état courant du rover sur /rover_status.
        La GUI PC (rover_gui.py) écoute ce topic et l'affiche dans le label de statut.
        """
        msg  = String()
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
        """
        Publie une commande Twist sur /cmd_vel et la mémorise pour l'odométrie.
        Toujours passer par cette méthode (pas directement pub_cmd_vel.publish)
        pour que last_linear et last_angular restent à jour.
        """
        msg = Twist()
        msg.linear.x  = float(linear)
        msg.angular.z = float(angular)
        self.pub_cmd_vel.publish(msg)
        # Mémorisation pour l'intégration odométrique dans _update_odometry()
        self.last_linear  = linear
        self.last_angular = angular

    @staticmethod
    def _normalize_angle(angle: float) -> float:
        """
        Ramène un angle dans [-π, π].
        Nécessaire car theta peut dépasser ces bornes après plusieurs rotations.
        atan2(sin(a), cos(a)) est le moyen le plus propre de normaliser.
        """
        return math.atan2(math.sin(angle), math.cos(angle))


def main(args=None):
    rclpy.init(args=args)
    node = AutonomousNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Arrêt par Ctrl+C')
    finally:
        node.publish_cmd(0.0, 0.0)  # sécurité : stoppe les moteurs avant de quitter
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
