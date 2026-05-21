# ══════════════════════════════════════════════════════════════════════════════
# autonomous_node.py — Raspberry Pi  (LifecycleNode)
#
# Navigation autonome case par case sur une grille 8×8.
# Algorithme greedy BFS + backtracking AMBUSH.
#
# GRILLE :
#   8 lignes × 8 colonnes (6×6 navigables + 1 rangée bordure tout autour)
#   CELL_ROW_MM=833, CELL_COL_MM=833  → terrain 5m×5m
#   Départ/cible : envoyés par le GUI via /rover/nav_goal — valeurs ci-dessous = défauts
#   Départ : (row=1, col=1)  Cible ArUco : (row=6, col=6)
#
# MACHINE D'ÉTATS :
#   IDLE → (nav_goal reçu) → EXPLORING → RETURNING → DONE
#   Sous-états de mouvement : ROTATING → MOVING → IDLE → (nav_step)
#
# TOPICS ÉCOUTÉS (uniquement en Active) :
#   /rover/nav_goal  Int32MultiArray    [start_row, start_col, target_row, target_col]
#   /rover/pose      Float32MultiArray  [x_mm, y_mm, theta_rad]
#   /rover/grid_pos  Int32MultiArray    [col, row]
#   /aruco_detected  Float32MultiArray  [found, id, cx, cy, area]
#   /ultrasonic      Float32MultiArray  [d1..d5] en mm
#
# TOPICS PUBLIÉS :
#   /rover/cmd_vel    geometry_msgs/Twist
#   /rover/grid_state Int32MultiArray  [64 valeurs]  — état de chaque case (row-major)
# ══════════════════════════════════════════════════════════════════════════════

import math
from collections import deque
from enum import Enum, auto
from math import atan2, cos, floor, pi, sin, sqrt

import rclpy
from geometry_msgs.msg import Twist
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn
from std_msgs.msg import Float32MultiArray, Int32MultiArray, String

# ── Grille ────────────────────────────────────────────────────────────────────
GRID_ROWS    = 8
GRID_COLS    = 8
CELL_ROW_MM  = 833
CELL_COL_MM  = 833
CELL_DIAG_MM = sqrt(CELL_ROW_MM**2 + CELL_COL_MM**2)   # ≈ 1178 mm

UNDISCOVERED = 0
FREE         = 1
OBSTACLE     = 2
AMBUSH       = 3
CELL_BORDER  = 4

START  = (1, 1)    # (row, col) — défaut GUI : haut-gauche
TARGET = (6, 6)    # (row, col) — défaut GUI : bas-droite (ArUco)

# Priorité Chebyshev : max_dist = max(|1-6|, |1-6|) = 5
_MAX_DIST = 5

# ── Capteurs ultrasoniques ────────────────────────────────────────────────────
# (sx_mm, sy_mm, angle_rad, seuil_mm) — repère rover : x=avant, y=gauche
SENSORS = [
    ( 40,  +50, math.radians(+30), 2400),   # d1 — avant-gauche
    ( 40,    0, math.radians(  0), 2400),   # d2 — avant-centre
    ( 40,  -50, math.radians(-30), 2400),   # d3 — avant-droite
    (-135, +150, math.radians(+90),  800),  # d4 — latéral gauche
    (-135, -150, math.radians(-90),  800),  # d5 — latéral droit
]
HIT_COUNT_REQUIRED = 2   # lectures consécutives avant de marquer OBSTACLE (debounce)

# ── Mouvement case à case ─────────────────────────────────────────────────────
ANGLE_TOL_RAD       = 0.08   # rad — ≈ 4.6° — tolérance fin de rotation
ARRIVAL_TOL_MM      = 100    # mm — tolérance arrivée centre de case (navigation grille)
ARRIVAL_TOL_FINE_MM = 40     # mm — tolérance fine alignment (alignement bouteille)
KP_ROT         = 1.5     # gain P rotation
MAX_ROT_SPEED  = 0.8     # rad/s
DECEL_START_MM = 500     # distance à partir de laquelle on commence à freiner
MIN_SPEED      = 0.07    # m/s — vitesse minimale d'approche
MAX_SPEED      = 0.25    # m/s
KA             = 0.5     # gain correction angulaire pendant MOVING

AMBUSH_LIMIT   = 5       # AMBUSH consécutifs max avant arrêt d'urgence

# ── Timeouts mouvement ────────────────────────────────────────────────────────
# Chaque timeout démarre depuis _move_start_time, réinitialisé au passage ROTATING→MOVING.
# Les deux phases ont donc leur propre budget de temps indépendant.
TIMEOUT_ROTATE_S    =  7.0   # s — rotation max (180° à 0.8 rad/s ≈ 5 s + marge)
TIMEOUT_MOVE_S      = 12.0   # s — déplacement case (diagonale 1155 mm ≈ 5 s + marge)
TIMEOUT_FINE_MOVE_S =  8.0   # s — fine alignment (courte distance, prend du temps à MIN_SPEED)
TIMEOUT_ARUCO_S     = 25.0   # s — approche ArUco totale (avance + recul)
STALE_POSE_S        =  1.0   # s — odométrie stale → arrêt d'urgence

# ── Filtrage ultrasonique ─────────────────────────────────────────────────────
US_WINDOW_SIZE = 3       # taille fenêtre médiane (= 300 ms à 10 Hz)
FRONT_STOP_MM  = 300     # arrêt d'urgence si obstacle devant < cette valeur (en MOVING)
_FRONT_IDX     = {0, 1, 2}   # indices d1, d2, d3 = capteurs avant

# ── 8 directions de navigation ────────────────────────────────────────────────
_DIRS8 = [(-1,-1),(-1,0),(-1,1),(0,-1),(0,1),(1,-1),(1,0),(1,1)]

# ── Séquence bras pour ramasser la bouteille ─────────────────────────────────
# Chaque étape : (durée en ticks @ 10 Hz, [z, y, pince, speed, dump, bin_dir])
#   z=−1 descend, z=+1 monte  |  pince=−100 ouvert, pince=+100 fermé
ARM_PICKUP_SEQ = [
    (10, [ 0.0, 0.0, -100.0, 1.0, 0.0, 0.0]),  # ouvrir pince
    (35, [-1.0, 0.0, -100.0, 1.0, 0.0, 0.0]),  # descendre bras
    (15, [ 0.0, 0.0,  100.0, 1.0, 0.0, 0.0]),  # fermer pince (saisir)
    (35, [ 1.0, 0.0,  100.0, 1.0, 0.0, 0.0]),  # monter bras
    ( 5, [ 0.0, 0.0,  100.0, 1.0, 0.0, 0.0]),  # maintien pince fermée
]

# ── Approche visuelle ArUco ────────────────────────────────────────────────────
ARUCO_AREA_THRESHOLD = 5000.0   # px² à ~1 m — à calibrer sur le rover réel
ARUCO_APPROACH_SPEED = 0.05     # m/s — vitesse d'approche visuelle lente
ARUCO_MAX_ADVANCE_MM = 1500.0   # mm — sécurité : distance max sans seuil aire
ARUCO_SEARCH_SPEED    = 0.15          # rad/s — rotation lente de recherche ArUco
ARUCO_SEARCH_HALF_ARC = math.pi / 2  # rad — balayage ±90° autour de l'orientation initiale


class _Mission(Enum):
    IDLE           = auto()
    EXPLORING      = auto()
    ARUCO_APPROACH = auto()   # approche visuelle ArUco + recul
    COLLECTING     = auto()   # naviguer vers bouteille + séquence bras
    RETURNING      = auto()
    DONE           = auto()


class _ArUcoPhase(Enum):
    SEARCHING   = auto()   # rotation lente jusqu'à trouver l'ArUco
    APPROACHING = auto()   # avance lentement, surveille l'aire
    BACKING_UP  = auto()   # recule de la même distance


class _Move(Enum):
    IDLE     = auto()
    ROTATING = auto()
    MOVING   = auto()


class AutonomousNode(LifecycleNode):

    def __init__(self):
        super().__init__('autonomous_node')

        self._pub_cmd        = None
        self._pub_grid_state = None
        self._pub_status     = None
        self._pub_arm        = None
        self._subs           = []
        self._timer          = None
        self._status_timer   = None

        # Horodatages pour détecter les données stales
        self._last_pose_time = None
        self._last_us_time   = None
        self._last_reason    = '—'

        # Pose courante (mise à jour par /rover/pose)
        self._x     = 0.0
        self._y     = 0.0
        self._theta = 0.0
        self._start = START        # mis à jour par /rover/nav_goal [start_r, start_c, tgt_r, tgt_c]
        self._row   = START[0]
        self._col   = START[1]

        # Grille et navigation
        self._cells         = []
        self._priority      = []
        self._path_stack    = []   # historique pour backtracking AMBUSH
        self._transit_path  = []   # chemin de transit FREE en cours

        # États
        self._mission   = _Mission.IDLE
        self._move      = _Move.IDLE
        self._nav_goal  = TARGET   # mis à jour par /rover/nav_goal

        # Cible du mouvement courant
        self._tgt_row      = START[0]
        self._tgt_col      = START[1]
        self._heading_tgt  = 0.0
        self._move_forward = True

        # ArUco
        self._aruco_found            = False
        self._aruco_area             = 0.0
        self._aruco_phase            = _ArUcoPhase.APPROACHING
        self._aruco_approach_heading = 0.0
        self._aruco_start_x          = 0.0
        self._aruco_start_y          = 0.0
        self._aruco_advance_dist     = 0.0
        self._search_step            = 0     # 0=vers +90°, 1=vers -90°, 2=épuisé
        self._search_origin_theta    = 0.0   # θ au début de la phase SEARCHING

        # Bouteille
        # _bottle_offset : (rel_x_mm, rel_y_mm) relatif au centre ArUco — envoyé par GUI
        # _bottle_world  : (x_mm, y_mm) en coordonnées arène, calculé depuis nav_goal
        self._bottle_offset = None
        self._bottle_world  = None
        self._collecting    = False  # True pendant l'exécution de la séquence bras
        self._arm_seq_idx   = 0
        self._arm_seq_ticks = 0

        # Navigation continue (fine alignment)
        self._use_world_target = False   # True = _do_move vise (_tgt_x, _tgt_y) au lieu du centre de case
        self._tgt_x = 0.0
        self._tgt_y = 0.0

        # Timeout mouvement
        self._move_start_time = None

        # Filtrage US
        self._us_hit     = [0] * len(SENSORS)
        self._us_windows = [[] for _ in range(len(SENSORS))]

        self._ambush_streak = 0

    # ── Lifecycle ──────────────────────────────────────────────────────────────

    def on_configure(self, state):
        self._pub_cmd        = self.create_publisher(Twist,             '/rover/cmd_vel',    10)
        self._pub_grid_state = self.create_publisher(Int32MultiArray,  '/rover/grid_state', 10)
        self._pub_status     = self.create_publisher(String,           '/rover_status',     10)
        self._pub_arm        = self.create_publisher(Float32MultiArray, '/rover/arm_cmd',   10)
        self.get_logger().info('autonomous_node configuré')
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state):
        self._reset()
        self._subs = [
            self.create_subscription(
                Float32MultiArray, '/rover/pose',       self._pose_cb,     10),
            self.create_subscription(
                Int32MultiArray,   '/rover/grid_pos',   self._grid_cb,     10),
            self.create_subscription(
                Float32MultiArray, '/aruco_detected',   self._aruco_cb,    10),
            self.create_subscription(
                Float32MultiArray, '/ultrasonic',       self._us_cb,       10),
            self.create_subscription(
                Int32MultiArray,   '/rover/nav_goal',    self._nav_goal_cb,  10),
            self.create_subscription(
                Float32MultiArray, '/rover/bottle_pos',  self._bottle_cb,    10),
        ]
        self._timer        = self.create_timer(0.1, self._tick)
        self._status_timer = self.create_timer(0.5, self._publish_status)
        self.get_logger().info('autonomous_node actif — en attente de nav_goal')
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state):
        self._stop()
        if self._timer:
            self.destroy_timer(self._timer)
            self._timer = None
        if self._status_timer:
            self.destroy_timer(self._status_timer)
            self._status_timer = None
        for s in self._subs:
            self.destroy_subscription(s)
        self._subs       = []
        self._mission    = _Mission.IDLE
        self._last_reason = 'node désactivé'
        self._publish_status()
        self.get_logger().info('autonomous_node inactif — moteurs stoppés')
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state):
        if self._pub_cmd:
            self.destroy_publisher(self._pub_cmd)
            self._pub_cmd = None
        if self._pub_grid_state:
            self.destroy_publisher(self._pub_grid_state)
            self._pub_grid_state = None
        if self._pub_status:
            self.destroy_publisher(self._pub_status)
            self._pub_status = None
        if self._pub_arm:
            self.destroy_publisher(self._pub_arm)
            self._pub_arm = None
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state):
        self._stop()
        return TransitionCallbackReturn.SUCCESS

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _pose_cb(self, msg: Float32MultiArray):
        if len(msg.data) >= 3:
            self._x, self._y, self._theta = float(msg.data[0]), float(msg.data[1]), float(msg.data[2])
            self._last_pose_time = self.get_clock().now()

    def _grid_cb(self, msg: Int32MultiArray):
        if len(msg.data) >= 2:
            self._col, self._row = int(msg.data[0]), int(msg.data[1])

    def _aruco_cb(self, msg: Float32MultiArray):
        if len(msg.data) < 1:
            return
        if len(msg.data) >= 5:
            self._aruco_area = float(msg.data[4])
        if msg.data[0] > 0.5 and not self._aruco_found:
            self._aruco_found = True
            self._recalc_priorities(self._nav_goal[0], self._nav_goal[1])
            self.get_logger().info('ArUco détecté — priorités recalculées vers nav_goal')

    def _us_cb(self, msg: Float32MultiArray):
        if len(msg.data) < len(SENSORS) or self._mission == _Mission.IDLE:
            return
        self._last_us_time = self.get_clock().now()

        for i, (sx, sy, sa, seuil) in enumerate(SENSORS):
            raw = msg.data[i]

            # Valeur hors plage physique → vider fenêtre et debounce
            if not (10.0 < raw < seuil):
                self._us_windows[i].clear()
                self._us_hit[i] = 0
                continue

            # Fenêtre glissante + médiane (atténue les pics parasites)
            win = self._us_windows[i]
            win.append(raw)
            if len(win) > US_WINDOW_SIZE:
                win.pop(0)
            d = sorted(win)[len(win) // 2]

            # Arrêt d'urgence : capteur avant très proche pendant MOVING
            if i in _FRONT_IDX and d < FRONT_STOP_MM and self._move == _Move.MOVING:
                self._stop()
                self._move = _Move.IDLE
                self._transit_path.clear()
                self._last_reason = f'ARRÊT URGENCE d{i + 1}={d:.0f}mm'
                self.get_logger().warn(self._last_reason)
                continue

            # Debounce → marquer obstacle
            self._us_hit[i] += 1
            if self._us_hit[i] >= HIT_COUNT_REQUIRED:
                self._mark_obstacle(sx, sy, sa, d)

    def _nav_goal_cb(self, msg: Int32MultiArray):
        if len(msg.data) < 2:
            return
        if len(msg.data) >= 4:
            # Format 4 valeurs : [start_row, start_col, target_row, target_col]
            sr, sc = int(msg.data[0]), int(msg.data[1])
            row, col = int(msg.data[2]), int(msg.data[3])
            if not (0 <= sr < GRID_ROWS and 0 <= sc < GRID_COLS):
                self.get_logger().warn(f'start hors grille : ({sr},{sc}) — ignoré')
                return
            self._start = (sr, sc)
            self._row, self._col = sr, sc
        else:
            # Format 2 valeurs (rétrocompatibilité) : [target_row, target_col]
            row, col = int(msg.data[0]), int(msg.data[1])
        if not (0 <= row < GRID_ROWS and 0 <= col < GRID_COLS):
            self.get_logger().warn(f'nav_goal hors grille : ({row},{col}) — ignoré')
            return
        self._nav_goal    = (row, col)
        self._last_reason = f'nav_goal ({row},{col})'
        self._compute_bottle_world()
        self._reset()
        self._recalc_priorities(row, col)
        self._mission = _Mission.EXPLORING
        self.get_logger().info(
            f'nav_goal reçu — départ ({self._start[0]},{self._start[1]}) '
            f'cible ({row},{col}) — EXPLORING'
        )

    # ── Boucle principale 10 Hz ───────────────────────────────────────────────

    def _tick(self):
        if self._mission in (_Mission.IDLE, _Mission.DONE):
            return
        if self._mission == _Mission.COLLECTING and self._collecting:
            self._arm_pickup_tick()
            return

        # Odométrie stale → arrêt d'urgence (rover aveugle)
        if self._last_pose_time is not None:
            age = (self.get_clock().now() - self._last_pose_time).nanoseconds / 1e9
            if age > STALE_POSE_S:
                self._stop()
                self._move = _Move.IDLE
                self._last_reason = f'POSE STALE {age:.1f}s — arrêt autonome'
                self.get_logger().error(self._last_reason)
                self._mission = _Mission.DONE
                return

        if self._mission == _Mission.ARUCO_APPROACH:
            self._aruco_approach_tick()
            return
        if self._move == _Move.ROTATING:
            self._do_rotate()
        elif self._move == _Move.MOVING:
            self._do_move()
        else:
            self._nav_step()

    # ── Navigation case par case ───────────────────────────────────────────────

    def _nav_step(self):
        # Marquer case courante FREE si découverte
        if self._cells[self._row][self._col] == UNDISCOVERED:
            self._cells[self._row][self._col] = FREE
            self._publish_grid_state()

        # Vérifier fin de mission
        if self._mission == _Mission.EXPLORING:
            tr, tc = self._nav_goal
            cheby = max(abs(self._row - tr), abs(self._col - tc))
            if cheby == 0:
                # Atterri dans la case ArUco elle-même — passer directement
                self._transit_path.clear()
                self._mission = _Mission.COLLECTING if self._bottle_world else _Mission.RETURNING
                self._last_reason = 'cible ArUco atteinte directement — ' + self._mission.name
                return
            elif cheby == 1:
                # Case adjacente → approche visuelle ArUco
                self._transit_path.clear()
                self._move = _Move.IDLE
                aruco_x = tc * CELL_COL_MM + CELL_COL_MM / 2.0
                aruco_y = tr * CELL_ROW_MM + CELL_ROW_MM / 2.0
                self._aruco_approach_heading = atan2(aruco_x - self._x, aruco_y - self._y)
                self._aruco_start_x  = self._x
                self._aruco_start_y  = self._y
                self._aruco_advance_dist = 0.0
                self._move_start_time = self.get_clock().now()
                self._mission = _Mission.ARUCO_APPROACH
                # Marquer la case bouteille OBSTACLE si déjà connue
                if self._bottle_world is not None:
                    br = max(0, min(GRID_ROWS - 1, int(floor(self._bottle_world[1] / CELL_ROW_MM))))
                    bc = max(0, min(GRID_COLS - 1, int(floor(self._bottle_world[0] / CELL_COL_MM))))
                    if self._cells[br][bc] not in (CELL_BORDER, OBSTACLE):
                        self._cells[br][bc] = OBSTACLE
                        self._publish_grid_state()
                if self._aruco_found:
                    self._aruco_phase = _ArUcoPhase.APPROACHING
                    self._last_reason = f'adjacent ArUco ({tr},{tc}) — ARUCO_APPROACH direct'
                else:
                    self._aruco_phase         = _ArUcoPhase.SEARCHING
                    self._search_step         = 0
                    self._search_origin_theta = self._theta
                    self._last_reason = f'adjacent ArUco ({tr},{tc}) — ARUCO_APPROACH recherche'
                self.get_logger().info(self._last_reason)
                return

        if self._mission == _Mission.COLLECTING:
            bx, by = self._bottle_world
            dist_to_bottle = sqrt((self._x - bx)**2 + (self._y - by)**2)
            if dist_to_bottle < ARRIVAL_TOL_FINE_MM:
                self._stop()
                self._collecting    = True
                self._arm_seq_idx   = 0
                self._arm_seq_ticks = 0
                self._last_reason   = 'bouteille atteinte — séquence bras'
                self.get_logger().info('Bouteille atteinte — démarrage séquence bras')
                return
            # Case adjacente à la bouteille → fine alignment (évite d'entrer dans sa case)
            bottle_row = int(floor(by / CELL_ROW_MM))
            bottle_col = int(floor(bx / CELL_COL_MM))
            chebyshev  = max(abs(self._row - bottle_row), abs(self._col - bottle_col))
            if chebyshev <= 1:
                self._start_move_to_world(bx, by)
                return

        if self._mission == _Mission.RETURNING:
            if self._row == self._start[0] and self._col == self._start[1]:
                self._last_reason = 'mission terminée'
                self.get_logger().info('Retour au départ — DONE')
                self._mission = _Mission.DONE
                self._stop()
                return

        # Suivre le transit en cours
        if self._transit_path:
            nr, nc = self._transit_path.pop(0)
            self._start_move_to(nr, nc)
            return

        # Planification BFS
        if self._mission == _Mission.EXPLORING:
            path = self._bfs_best_undiscovered(self._row, self._col)
        elif self._mission == _Mission.COLLECTING:
            bx, by = self._bottle_world
            br = int(floor(by / CELL_ROW_MM))
            bc = int(floor(bx / CELL_COL_MM))
            path = self._bfs_to(self._row, self._col, br, bc)
        else:
            # RETURNING : chemin optimal par cases déjà visitées (FREE), fallback général
            path = self._bfs_to(self._row, self._col, self._start[0], self._start[1], free_only=True)
            if path is None or len(path) < 2:
                path = self._bfs_to(self._row, self._col, self._start[0], self._start[1])

        if path is None or len(path) < 2:
            if self._mission == _Mission.COLLECTING:
                self._last_reason = 'aucun chemin bouteille — retour direct'
                self.get_logger().error('Aucun chemin vers bouteille — RETURNING')
                self._mission = _Mission.RETURNING
                return
            if self._mission == _Mission.RETURNING:
                self._last_reason = 'aucun chemin retour — bloqué'
                self.get_logger().error('Aucun chemin retour — arrêt')
                self._mission = _Mission.DONE
                self._stop()
                return

            # AMBUSH
            self._cells[self._row][self._col] = AMBUSH
            self._publish_grid_state()
            self._ambush_streak += 1
            self._last_reason = f'AMBUSH ({self._row},{self._col}) streak={self._ambush_streak}'
            self.get_logger().warn(self._last_reason)
            if self._ambush_streak >= AMBUSH_LIMIT:
                self._last_reason = 'rover bloqué — arrêt autonome'
                self.get_logger().error('Rover bloqué — arrêt autonome')
                self._mission = _Mission.DONE
                self._stop()
                return
            if self._path_stack:
                pr, pc = self._path_stack.pop()
                self._start_move_to(pr, pc)
            else:
                self._mission = _Mission.DONE
                self._stop()
            return

        self._ambush_streak = 0
        self._path_stack.append((self._row, self._col))

        if len(path) == 2:
            nr, nc = path[1]
            self._start_move_to(nr, nc)
        else:
            # Transit multi-cases à travers des cases FREE
            self._transit_path = list(path[1:])
            nr, nc = self._transit_path.pop(0)
            self._start_move_to(nr, nc)

    # ── Sous-états de mouvement ────────────────────────────────────────────────

    def _start_move_to(self, row: int, col: int):
        self._tgt_row = row
        self._tgt_col = col

        dr = row - self._row
        dc = col - self._col
        heading = atan2(dc, dr)   # 0° = avancer (rows croissants)

        delta = _norm(heading - self._theta)
        if abs(delta) <= pi / 2:
            self._heading_tgt  = heading
            self._move_forward = True
        else:
            self._heading_tgt  = _norm(heading + pi)
            self._move_forward = False

        self._use_world_target = False
        self._move_start_time  = self.get_clock().now()
        self._move = _Move.ROTATING
        self.get_logger().info(
            f'→ ({row},{col})  cap={math.degrees(self._heading_tgt):.1f}°  '
            f'{"FWD" if self._move_forward else "REV"}'
        )

    def _start_move_to_world(self, wx: float, wy: float):
        """Fine alignment : navigation vers des coordonnées monde exactes (mm)."""
        self._tgt_x = wx
        self._tgt_y = wy
        dx = wx - self._x
        dy = wy - self._y
        heading = atan2(dx, dy)
        delta = _norm(heading - self._theta)
        if abs(delta) <= pi / 2:
            self._heading_tgt  = heading
            self._move_forward = True
        else:
            self._heading_tgt  = _norm(heading + pi)
            self._move_forward = False
        self._use_world_target = True
        self._move_start_time  = self.get_clock().now()
        self._move = _Move.ROTATING
        self.get_logger().info(
            f'→ fine align ({wx:.0f},{wy:.0f}) mm  '
            f'cap={math.degrees(self._heading_tgt):.1f}°'
        )

    def _do_rotate(self):
        if self._move_start_time is not None:
            elapsed = (self.get_clock().now() - self._move_start_time).nanoseconds / 1e9
            if elapsed > TIMEOUT_ROTATE_S:
                self._stop()
                self._move_start_time = self.get_clock().now()   # reset : MOVING a son propre budget
                self._move = _Move.MOVING
                self._last_reason = f'TIMEOUT rotation {elapsed:.1f}s — passe MOVING de force'
                self.get_logger().warn(self._last_reason)
                return

        delta = _norm(self._heading_tgt - self._theta)
        if abs(delta) < ANGLE_TOL_RAD:
            self._stop()
            self._move_start_time = self.get_clock().now()   # reset : MOVING a son propre budget
            self._move = _Move.MOVING
            return
        spd = max(-MAX_ROT_SPEED, min(MAX_ROT_SPEED, KP_ROT * delta))
        self._cmd(0.0, spd)

    def _do_move(self):
        if self._move_start_time is not None:
            elapsed = (self.get_clock().now() - self._move_start_time).nanoseconds / 1e9
            tmo = TIMEOUT_FINE_MOVE_S if self._use_world_target else TIMEOUT_MOVE_S
            if elapsed > tmo:
                self._stop()
                self._move = _Move.IDLE
                if self._use_world_target:
                    # Fine alignment — revenir au BFS grille
                    self._use_world_target = False
                    self._last_reason = f'TIMEOUT fine align {elapsed:.1f}s — retour BFS grille'
                else:
                    # Déplacement case — marquer AMBUSH, replanifier
                    self._cells[self._tgt_row][self._tgt_col] = AMBUSH
                    self._publish_grid_state()
                    self._last_reason = f'TIMEOUT move {elapsed:.1f}s — AMBUSH ({self._tgt_row},{self._tgt_col})'
                self.get_logger().warn(self._last_reason)
                return

        if self._use_world_target:
            x_c, y_c = self._tgt_x, self._tgt_y
            tol = ARRIVAL_TOL_FINE_MM
        else:
            # Stopper si la case cible est devenue un obstacle
            if self._cells[self._tgt_row][self._tgt_col] == OBSTACLE:
                self._stop()
                self._move = _Move.IDLE
                return
            x_c = self._tgt_col * CELL_COL_MM + CELL_COL_MM / 2.0
            y_c = self._tgt_row * CELL_ROW_MM + CELL_ROW_MM / 2.0
            tol = ARRIVAL_TOL_MM

        dist = sqrt((self._x - x_c)**2 + (self._y - y_c)**2)

        if dist < tol:
            self._stop()
            self._move = _Move.IDLE
            return

        if dist >= DECEL_START_MM:
            lin = MAX_SPEED
        else:
            # Rampe linéaire : MAX_SPEED à DECEL_START_MM → MIN_SPEED à ARRIVAL_TOL_MM
            t = (dist - ARRIVAL_TOL_MM) / (DECEL_START_MM - ARRIVAL_TOL_MM)
            lin = MIN_SPEED + (MAX_SPEED - MIN_SPEED) * max(0.0, t)
        if not self._move_forward:
            lin = -lin

        ang = KA * _norm(self._heading_tgt - self._theta)
        self._cmd(lin, ang)

    # ── Obstacles ultrasoniques ───────────────────────────────────────────────

    def _mark_obstacle(self, sx: float, sy: float, sa: float, d: float):
        th = self._theta
        wx = self._x + sx * sin(th) - sy * cos(th)
        wy = self._y + sx * cos(th) + sy * sin(th)
        ox = wx + d * sin(th - sa)
        oy = wy + d * cos(th - sa)

        obs_r = int(floor(oy / CELL_ROW_MM))
        obs_c = int(floor(ox / CELL_COL_MM))

        if not (0 <= obs_r < GRID_ROWS and 0 <= obs_c < GRID_COLS):
            return
        if obs_r == self._row and obs_c == self._col:
            return   # ne pas marquer la case actuelle
        if self._cells[obs_r][obs_c] in (CELL_BORDER, OBSTACLE):
            return

        self._cells[obs_r][obs_c] = OBSTACLE
        self._publish_grid_state()
        self._last_reason = f'obstacle ({obs_r},{obs_c}) à {d:.0f}mm'
        self.get_logger().info(self._last_reason)

        # Invalider le transit si l'obstacle y tombe
        if any(r == obs_r and c == obs_c for r, c in self._transit_path):
            self._transit_path.clear()
            if self._path_stack:
                self._path_stack.pop()   # annuler le push du démarrage du transit
            self._stop()
            self._move = _Move.IDLE
            self.get_logger().warn(f'Obstacle sur transit ({obs_r},{obs_c}) — replanification')

    # ── BFS ───────────────────────────────────────────────────────────────────

    def _bfs_best_undiscovered(self, sr: int, sc: int):
        """Chemin vers la case UNDISCOVERED atteignable avec la plus haute priorité."""
        best_path = None
        best_prio = -1

        visited = [[False] * GRID_COLS for _ in range(GRID_ROWS)]
        visited[sr][sc] = True
        q = deque()
        q.append([(sr, sc)])

        while q:
            path = q.popleft()
            r, c = path[-1]

            state = self._cells[r][c]

            if state == UNDISCOVERED and (r, c) != (sr, sc):
                prio = self._priority[r][c]
                if prio > best_prio:
                    best_prio = prio
                    best_path = path
                continue   # ne pas traverser à travers UNDISCOVERED

            if state not in (FREE, UNDISCOVERED):
                continue

            for dr, dc in _DIRS8:
                nr, nc = r + dr, c + dc
                if not (0 <= nr < GRID_ROWS and 0 <= nc < GRID_COLS):
                    continue
                if visited[nr][nc]:
                    continue
                ns = self._cells[nr][nc]
                if ns in (CELL_BORDER, OBSTACLE, AMBUSH):
                    continue
                if not _diag_clear(self._cells, r, c, dr, dc):
                    continue
                visited[nr][nc] = True
                q.append(path + [(nr, nc)])

        return best_path

    def _bfs_to(self, sr: int, sc: int, tr: int, tc: int, free_only: bool = False):
        """Chemin de (sr,sc) à (tr,tc) sur la grille connue.
        free_only=True : uniquement les cases FREE (retour optimal par cases visitées)."""
        if sr == tr and sc == tc:
            return [(sr, sc), (tr, tc)]   # déjà sur place — longueur 2 pour éviter AMBUSH

        visited = [[False] * GRID_COLS for _ in range(GRID_ROWS)]
        visited[sr][sc] = True
        q = deque()
        q.append([(sr, sc)])

        while q:
            path = q.popleft()
            r, c = path[-1]
            if r == tr and c == tc:
                return path
            for dr, dc in _DIRS8:
                nr, nc = r + dr, c + dc
                if not (0 <= nr < GRID_ROWS and 0 <= nc < GRID_COLS):
                    continue
                if visited[nr][nc]:
                    continue
                ns = self._cells[nr][nc]
                if ns in (CELL_BORDER, OBSTACLE, AMBUSH):
                    continue
                if free_only and ns != FREE:
                    continue
                if not _diag_clear(self._cells, r, c, dr, dc):
                    continue
                visited[nr][nc] = True
                q.append(path + [(nr, nc)])

        return None

    # ── Priorités Chebyshev ────────────────────────────────────────────────────

    def _recalc_priorities(self, tr: int, tc: int):
        for r in range(GRID_ROWS):
            for c in range(GRID_COLS):
                if self._cells[r][c] == CELL_BORDER:
                    self._priority[r][c] = -1
                else:
                    cheby = max(abs(r - tr), abs(c - tc))
                    self._priority[r][c] = _MAX_DIST - cheby

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _reset(self):
        self._cells = [[UNDISCOVERED] * GRID_COLS for _ in range(GRID_ROWS)]
        for r in range(GRID_ROWS):
            for c in range(GRID_COLS):
                if r == 0 or r == GRID_ROWS - 1 or c == 0 or c == GRID_COLS - 1:
                    self._cells[r][c] = CELL_BORDER

        self._priority = [[0] * GRID_COLS for _ in range(GRID_ROWS)]
        self._recalc_priorities(self._nav_goal[0], self._nav_goal[1])

        self._path_stack    = []
        self._transit_path  = []
        self._move          = _Move.IDLE
        self._mission       = _Mission.IDLE
        self._aruco_found            = False
        self._aruco_area             = 0.0
        self._aruco_phase            = _ArUcoPhase.APPROACHING
        self._aruco_approach_heading = 0.0
        self._aruco_start_x          = 0.0
        self._aruco_start_y          = 0.0
        self._aruco_advance_dist     = 0.0
        self._search_step            = 0
        self._search_origin_theta    = 0.0
        self._ambush_streak = 0
        self._collecting       = False
        self._arm_seq_idx      = 0
        self._arm_seq_ticks    = 0
        self._use_world_target = False
        self._move_start_time  = None
        self._us_hit     = [0] * len(SENSORS)
        self._us_windows = [[] for _ in range(len(SENSORS))]
        self._row, self._col = self._start

        self.get_logger().info(
            f'Grille réinitialisée — départ ({self._start[0]},{self._start[1]}), '
            f'cible ({self._nav_goal[0]},{self._nav_goal[1]})'
        )

    def _publish_status(self):
        if self._pub_status is None:
            return
        now = self.get_clock().now()

        def _age(t):
            if t is None:
                return '—'
            return f'{(now - t).nanoseconds / 1e9:.1f}s'

        msg = String()
        msg.data = (
            f'{self._mission.name} · {self._move.name} · '
            f'({self._row},{self._col})→({self._nav_goal[0]},{self._nav_goal[1]}) · '
            f'pose:{_age(self._last_pose_time)} us:{_age(self._last_us_time)} · '
            f'{self._last_reason}'
        )
        self._pub_status.publish(msg)

    def _publish_grid_state(self):
        if self._pub_grid_state is None or not self._cells:
            return
        flat = [self._cells[r][c] for r in range(GRID_ROWS) for c in range(GRID_COLS)]
        msg = Int32MultiArray()
        msg.data = flat
        self._pub_grid_state.publish(msg)

    def _bottle_cb(self, msg: Float32MultiArray):
        if len(msg.data) < 2:
            return
        self._bottle_offset = (float(msg.data[0]), float(msg.data[1]))
        self._compute_bottle_world()
        self.get_logger().info(
            f'Offset bouteille reçu : ({self._bottle_offset[0]:.0f},{self._bottle_offset[1]:.0f}) mm'
        )

    def _compute_bottle_world(self):
        """Calcule la position monde de la bouteille depuis nav_goal + offset relatif ArUco."""
        if self._bottle_offset is None:
            return
        aruco_row, aruco_col = self._nav_goal
        aruco_x = aruco_col * CELL_COL_MM + CELL_COL_MM / 2.0
        aruco_y = aruco_row * CELL_ROW_MM + CELL_ROW_MM / 2.0
        rel_x, rel_y = self._bottle_offset
        self._bottle_world = (aruco_x + rel_x, aruco_y + rel_y)
        self.get_logger().info(
            f'Position bouteille monde : ({self._bottle_world[0]:.0f},{self._bottle_world[1]:.0f}) mm'
        )
        # Marquer la case bouteille OBSTACLE dès que connue (sauf COLLECTING où on y entre)
        if self._cells and self._mission not in (_Mission.IDLE, _Mission.DONE, _Mission.COLLECTING):
            br = max(0, min(GRID_ROWS - 1, int(floor(self._bottle_world[1] / CELL_ROW_MM))))
            bc = max(0, min(GRID_COLS - 1, int(floor(self._bottle_world[0] / CELL_COL_MM))))
            if self._cells[br][bc] not in (CELL_BORDER, OBSTACLE):
                self._cells[br][bc] = OBSTACLE
                self._publish_grid_state()
                self.get_logger().info(f'Case bouteille ({br},{bc}) marquée OBSTACLE (ARUCO_APPROACH)')

    def _arm_pickup_tick(self):
        if self._arm_seq_idx >= len(ARM_PICKUP_SEQ):
            self._collecting  = False
            self._mission     = _Mission.RETURNING
            self._last_reason = 'bouteille saisie — retour'
            self.get_logger().info('Séquence bras terminée — RETURNING')
            return

        duration, cmd = ARM_PICKUP_SEQ[self._arm_seq_idx]
        self._arm_cmd(cmd)
        self._arm_seq_ticks += 1
        if self._arm_seq_ticks >= duration:
            self._arm_seq_idx   += 1
            self._arm_seq_ticks  = 0

    def _arm_cmd(self, cmd: list):
        if self._pub_arm is None:
            return
        msg = Float32MultiArray()
        msg.data = [float(v) for v in cmd]
        self._pub_arm.publish(msg)

    def _aruco_approach_tick(self):
        if self._move_start_time is not None:
            elapsed = (self.get_clock().now() - self._move_start_time).nanoseconds / 1e9
            if elapsed > TIMEOUT_ARUCO_S:
                self._stop()
                self._mission = _Mission.COLLECTING if self._bottle_world else _Mission.RETURNING
                self._last_reason = f'TIMEOUT ArUco {elapsed:.1f}s — {self._mission.name}'
                self.get_logger().warn(self._last_reason)
                return

        if self._aruco_phase == _ArUcoPhase.SEARCHING:
            if self._aruco_found:
                self._stop()
                self._aruco_approach_heading = self._theta
                self._aruco_start_x  = self._x
                self._aruco_start_y  = self._y
                self._aruco_advance_dist = 0.0
                self._aruco_phase = _ArUcoPhase.APPROACHING
                self._last_reason = 'ArUco trouvé en recherche — passage APPROACHING'
                self.get_logger().info(self._last_reason)
                return

            hi = _norm(self._search_origin_theta + ARUCO_SEARCH_HALF_ARC)
            lo = _norm(self._search_origin_theta - ARUCO_SEARCH_HALF_ARC)

            if self._search_step == 0:
                delta = _norm(hi - self._theta)
                if abs(delta) < ANGLE_TOL_RAD:
                    self._search_step = 1
                else:
                    self._cmd(0.0, ARUCO_SEARCH_SPEED if delta > 0 else -ARUCO_SEARCH_SPEED)
            elif self._search_step == 1:
                delta = _norm(lo - self._theta)
                if abs(delta) < ANGLE_TOL_RAD:
                    self._search_step = 2
                else:
                    self._cmd(0.0, ARUCO_SEARCH_SPEED if delta > 0 else -ARUCO_SEARCH_SPEED)
            else:
                self._stop()
                self._mission = _Mission.COLLECTING if self._bottle_world else _Mission.RETURNING
                self._last_reason = 'ArUco non trouvé après ±90° — ' + self._mission.name
                self.get_logger().warn(self._last_reason)
            return

        if self._aruco_phase == _ArUcoPhase.APPROACHING:
            # Rotation d'alignement vers l'ArUco avant d'avancer
            delta = _norm(self._aruco_approach_heading - self._theta)
            if abs(delta) > ANGLE_TOL_RAD:
                spd = max(-MAX_ROT_SPEED, min(MAX_ROT_SPEED, KP_ROT * delta))
                self._cmd(0.0, spd)
                return

            dist = sqrt((self._x - self._aruco_start_x)**2 +
                        (self._y - self._aruco_start_y)**2)

            stop_area = self._aruco_area > 0 and self._aruco_area >= ARUCO_AREA_THRESHOLD
            if stop_area or dist >= ARUCO_MAX_ADVANCE_MM:
                self._stop()
                self._aruco_advance_dist = dist
                self._aruco_phase = _ArUcoPhase.BACKING_UP
                self._last_reason = (
                    f'ArUco portée (aire={self._aruco_area:.0f}) avancé={dist:.0f}mm — recul'
                )
                self.get_logger().info(self._last_reason)
                return

            ang = KA * _norm(self._aruco_approach_heading - self._theta)
            self._cmd(ARUCO_APPROACH_SPEED, ang)

        else:  # BACKING_UP — recule jusqu'à la position de départ
            dist_to_start = sqrt((self._x - self._aruco_start_x)**2 +
                                 (self._y - self._aruco_start_y)**2)

            if dist_to_start <= ARRIVAL_TOL_FINE_MM:
                self._stop()
                # Relocaliser la case courante depuis la pose odométrique
                self._row = max(0, min(GRID_ROWS - 1, int(floor(self._y / CELL_ROW_MM))))
                self._col = max(0, min(GRID_COLS - 1, int(floor(self._x / CELL_COL_MM))))
                tr, tc = self._nav_goal
                if self._bottle_world is not None:
                    # Libérer la case bouteille (on va y rentrer pour ramasser)
                    br = max(0, min(GRID_ROWS - 1, int(floor(self._bottle_world[1] / CELL_ROW_MM))))
                    bc = max(0, min(GRID_COLS - 1, int(floor(self._bottle_world[0] / CELL_COL_MM))))
                    if self._cells[br][bc] == OBSTACLE and (br, bc) != (tr, tc):
                        self._cells[br][bc] = FREE
                    # Marquer la case ArUco OBSTACLE (ne pas percuter le marqueur)
                    if self._cells[tr][tc] not in (CELL_BORDER, OBSTACLE):
                        self._cells[tr][tc] = OBSTACLE
                    self._publish_grid_state()
                    self._mission = _Mission.COLLECTING
                    self._last_reason = 'recul ArUco terminé — COLLECTING'
                else:
                    # Marquer la case ArUco OBSTACLE pour le retour
                    if self._cells[tr][tc] not in (CELL_BORDER, OBSTACLE):
                        self._cells[tr][tc] = OBSTACLE
                        self._publish_grid_state()
                    self._mission = _Mission.RETURNING
                    self._last_reason = 'recul ArUco terminé — RETURNING'
                self.get_logger().info(self._last_reason)
                return

            # Reculer en maintenant le cap vers l'ArUco
            ang = KA * _norm(self._aruco_approach_heading - self._theta)
            self._cmd(-ARUCO_APPROACH_SPEED, ang)

    def _stop(self):
        self._cmd(0.0, 0.0)

    def _cmd(self, linear: float, angular: float):
        if self._pub_cmd is None:
            return
        msg = Twist()
        msg.linear.x  = float(linear)
        msg.angular.z = float(angular)
        self._pub_cmd.publish(msg)


# ── Fonctions pures (hors classe) ─────────────────────────────────────────────

def _norm(a: float) -> float:
    return atan2(sin(a), cos(a))


def _diag_clear(cells, r: int, c: int, dr: int, dc: int) -> bool:
    if dr == 0 or dc == 0:
        return True
    s1 = cells[r + dr][c]
    s2 = cells[r][c + dc]
    return s1 not in (OBSTACLE, CELL_BORDER) and s2 not in (OBSTACLE, CELL_BORDER)


def main(args=None):
    rclpy.init(args=args)
    node = AutonomousNode()
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
