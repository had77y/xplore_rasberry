# ══════════════════════════════════════════════════════════════════════════════
# autonomous_node.py — Raspberry Pi  (LifecycleNode)
#
# Navigation autonome case par case sur une grille 12×8.
# Algorithme greedy BFS + backtracking AMBUSH.
#
# GRILLE :
#   12 lignes × 8 colonnes (10×6 navigables + 1 rangée bordure tout autour)
#   CELL_ROW_MM=800, CELL_COL_MM=833
#   Départ : (row=10, col=6)  Cible ArUco : (row=1, col=1)
#
# MACHINE D'ÉTATS :
#   IDLE → EXPLORING → RETURNING → DONE
#   Sous-états de mouvement : ROTATING → MOVING → IDLE → (nav_step)
#
# TOPICS ÉCOUTÉS (uniquement en Active) :
#   /rover/pose      Float32MultiArray  [x_mm, y_mm, theta_rad]
#   /rover/grid_pos  Int32MultiArray    [col, row]
#   /aruco_detected  Float32MultiArray  [found, id, cx, cy, area]
#   /ultrasonic      Float32MultiArray  [d1..d5] en mm
#
# TOPICS PUBLIÉS :
#   /rover/cmd_vel   geometry_msgs/Twist
# ══════════════════════════════════════════════════════════════════════════════

import math
from collections import deque
from enum import Enum, auto
from math import atan2, cos, floor, pi, sin, sqrt

import rclpy
from geometry_msgs.msg import Twist
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn
from std_msgs.msg import Float32MultiArray, Int32MultiArray

# ── Grille ────────────────────────────────────────────────────────────────────
GRID_ROWS    = 12
GRID_COLS    = 8
CELL_ROW_MM  = 800
CELL_COL_MM  = 833
CELL_DIAG_MM = sqrt(CELL_ROW_MM**2 + CELL_COL_MM**2)   # ≈ 1155 mm

UNDISCOVERED = 0
FREE         = 1
OBSTACLE     = 2
AMBUSH       = 3
CELL_BORDER  = 4

START  = (10, 6)    # (row, col) — bas-droite
TARGET = (1,  1)    # (row, col) — haut-gauche (ArUco)

# Priorité Chebyshev : max_dist = max(|10-1|, |6-1|) = max(9, 5) = 9
_MAX_DIST = 9

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
ANGLE_TOL_RAD  = 0.08    # rad — ≈ 4.6° — tolérance fin de rotation
ARRIVAL_TOL_MM = 100     # mm — tolérance arrivée au centre de case
KP_ROT         = 1.5     # gain P rotation
MAX_ROT_SPEED  = 0.8     # rad/s
KP_LIN         = 0.0008  # gain P avancement (mm → m/s)
MIN_SPEED      = 0.10    # m/s
MAX_SPEED      = 0.25    # m/s
KA             = 0.5     # gain correction angulaire pendant MOVING

AMBUSH_LIMIT   = 5       # AMBUSH consécutifs max avant arrêt d'urgence

# ── 8 directions de navigation ────────────────────────────────────────────────
_DIRS8 = [(-1,-1),(-1,0),(-1,1),(0,-1),(0,1),(1,-1),(1,0),(1,1)]


class _Mission(Enum):
    IDLE      = auto()
    EXPLORING = auto()
    RETURNING = auto()
    DONE      = auto()


class _Move(Enum):
    IDLE     = auto()
    ROTATING = auto()
    MOVING   = auto()


class AutonomousNode(LifecycleNode):

    def __init__(self):
        super().__init__('autonomous_node')

        self._pub_cmd = None
        self._subs    = []
        self._timer   = None

        # Pose courante (mise à jour par /rover/pose)
        self._x     = 0.0
        self._y     = 0.0
        self._theta = 0.0
        self._row   = START[0]
        self._col   = START[1]

        # Grille et navigation
        self._cells         = []
        self._priority      = []
        self._path_stack    = []   # historique pour backtracking AMBUSH
        self._transit_path  = []   # chemin de transit FREE en cours

        # États
        self._mission = _Mission.IDLE
        self._move    = _Move.IDLE

        # Cible du mouvement courant
        self._tgt_row      = START[0]
        self._tgt_col      = START[1]
        self._heading_tgt  = 0.0
        self._move_forward = True

        # ArUco
        self._aruco_found = False

        # Debounce US
        self._us_hit = [0] * len(SENSORS)

        self._ambush_streak = 0

    # ── Lifecycle ──────────────────────────────────────────────────────────────

    def on_configure(self, state):
        self._pub_cmd = self.create_publisher(Twist, '/rover/cmd_vel', 10)
        self.get_logger().info('autonomous_node configuré')
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state):
        self._reset()
        self._subs = [
            self.create_subscription(
                Float32MultiArray, '/rover/pose',       self._pose_cb,  10),
            self.create_subscription(
                Int32MultiArray,   '/rover/grid_pos',   self._grid_cb,  10),
            self.create_subscription(
                Float32MultiArray, '/aruco_detected',   self._aruco_cb, 10),
            self.create_subscription(
                Float32MultiArray, '/ultrasonic',       self._us_cb,    10),
        ]
        self._timer   = self.create_timer(0.1, self._tick)
        self._mission = _Mission.EXPLORING
        self.get_logger().info('autonomous_node actif — EXPLORING')
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state):
        self._stop()
        if self._timer:
            self.destroy_timer(self._timer)
            self._timer = None
        for s in self._subs:
            self.destroy_subscription(s)
        self._subs    = []
        self._mission = _Mission.IDLE
        self.get_logger().info('autonomous_node inactif — moteurs stoppés')
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state):
        if self._pub_cmd:
            self.destroy_publisher(self._pub_cmd)
            self._pub_cmd = None
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state):
        self._stop()
        return TransitionCallbackReturn.SUCCESS

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _pose_cb(self, msg: Float32MultiArray):
        if len(msg.data) >= 3:
            self._x, self._y, self._theta = float(msg.data[0]), float(msg.data[1]), float(msg.data[2])

    def _grid_cb(self, msg: Int32MultiArray):
        if len(msg.data) >= 2:
            self._col, self._row = int(msg.data[0]), int(msg.data[1])

    def _aruco_cb(self, msg: Float32MultiArray):
        if len(msg.data) < 1:
            return
        if msg.data[0] > 0.5 and not self._aruco_found:
            self._aruco_found = True
            self._recalc_priorities(TARGET[0], TARGET[1])
            self.get_logger().info('ArUco détecté — priorités recalculées vers TARGET')

    def _us_cb(self, msg: Float32MultiArray):
        if len(msg.data) < len(SENSORS) or self._mission == _Mission.IDLE:
            return
        for i, (sx, sy, sa, seuil) in enumerate(SENSORS):
            d = msg.data[i]
            if 10.0 < d < seuil:
                self._us_hit[i] += 1
                if self._us_hit[i] >= HIT_COUNT_REQUIRED:
                    self._mark_obstacle(sx, sy, sa, d)
            else:
                self._us_hit[i] = 0

    # ── Boucle principale 10 Hz ───────────────────────────────────────────────

    def _tick(self):
        if self._mission in (_Mission.IDLE, _Mission.DONE):
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

        # Vérifier fin de mission
        if self._mission == _Mission.EXPLORING:
            if self._row == TARGET[0] and self._col == TARGET[1]:
                self.get_logger().info('Cible atteinte — RETURNING')
                self._mission = _Mission.RETURNING
                self._transit_path.clear()
                return   # prochain tick relancera _nav_step en RETURNING

        if self._mission == _Mission.RETURNING:
            if self._row == START[0] and self._col == START[1]:
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
        else:
            path = self._bfs_to(self._row, self._col, START[0], START[1])

        if path is None or len(path) < 2:
            if self._mission == _Mission.RETURNING:
                self.get_logger().error('Aucun chemin retour — arrêt')
                self._mission = _Mission.DONE
                self._stop()
                return

            # AMBUSH
            self._cells[self._row][self._col] = AMBUSH
            self._ambush_streak += 1
            self.get_logger().warn(
                f'AMBUSH ({self._row},{self._col}) streak={self._ambush_streak}'
            )
            if self._ambush_streak >= AMBUSH_LIMIT:
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

        self._move = _Move.ROTATING
        self.get_logger().info(
            f'→ ({row},{col})  cap={math.degrees(self._heading_tgt):.1f}°  '
            f'{"FWD" if self._move_forward else "REV"}'
        )

    def _do_rotate(self):
        delta = _norm(self._heading_tgt - self._theta)
        if abs(delta) < ANGLE_TOL_RAD:
            self._stop()
            self._move = _Move.MOVING
            return
        spd = max(-MAX_ROT_SPEED, min(MAX_ROT_SPEED, KP_ROT * delta))
        self._cmd(0.0, spd)

    def _do_move(self):
        # Stopper si la case cible est devenue un obstacle
        if self._cells[self._tgt_row][self._tgt_col] == OBSTACLE:
            self._stop()
            self._move = _Move.IDLE
            return

        x_c = self._tgt_col * CELL_COL_MM + CELL_COL_MM / 2.0
        y_c = self._tgt_row * CELL_ROW_MM + CELL_ROW_MM / 2.0
        dist = sqrt((self._x - x_c)**2 + (self._y - y_c)**2)

        if dist < ARRIVAL_TOL_MM:
            self._stop()
            self._move = _Move.IDLE
            return

        lin = max(MIN_SPEED, min(MAX_SPEED, KP_LIN * dist))
        if not self._move_forward:
            lin = -lin

        ang = KA * _norm(self._heading_tgt - self._theta)
        self._cmd(lin, ang)

    # ── Obstacles ultrasoniques ───────────────────────────────────────────────

    def _mark_obstacle(self, sx: float, sy: float, sa: float, d: float):
        th = self._theta
        wx = self._x + sx * cos(th) - sy * sin(th)
        wy = self._y + sx * sin(th) + sy * cos(th)
        ox = wx + d * cos(th + sa)
        oy = wy + d * sin(th + sa)

        obs_r = int(floor(oy / CELL_ROW_MM))
        obs_c = int(floor(ox / CELL_COL_MM))

        if not (0 <= obs_r < GRID_ROWS and 0 <= obs_c < GRID_COLS):
            return
        if self._cells[obs_r][obs_c] in (CELL_BORDER, OBSTACLE):
            return

        self._cells[obs_r][obs_c] = OBSTACLE
        self.get_logger().info(f'Obstacle marqué ({obs_r},{obs_c})')

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

    def _bfs_to(self, sr: int, sc: int, tr: int, tc: int):
        """Chemin de (sr,sc) à (tr,tc) sur la grille connue."""
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
                if self._cells[nr][nc] in (CELL_BORDER, OBSTACLE, AMBUSH):
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
        self._recalc_priorities(TARGET[0], TARGET[1])

        self._path_stack   = []
        self._transit_path = []
        self._move         = _Move.IDLE
        self._mission      = _Mission.IDLE
        self._aruco_found  = False
        self._ambush_streak = 0
        self._us_hit       = [0] * len(SENSORS)
        self._row, self._col = START

        self.get_logger().info(
            f'Grille réinitialisée — départ ({START[0]},{START[1]}), '
            f'cible ({TARGET[0]},{TARGET[1]})'
        )

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
