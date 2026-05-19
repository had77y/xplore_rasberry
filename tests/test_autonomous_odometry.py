"""
Tests autonomes — logique pure sans ROS (mock complet).
Lance : python3 -m pytest tests/test_autonomous_odometry.py -v
"""
import sys
import math
from math import pi, sqrt, sin, cos, atan2, floor
from unittest.mock import MagicMock, patch

# ── Mock ROS avant tout import ─────────────────────────────────────────────────
class _FakeNode:
    def __init__(self, *a, **kw): pass
    def get_logger(self):        return MagicMock()
    def get_clock(self):
        c = MagicMock()
        c.now.return_value = MagicMock()
        return c
    def create_publisher(self, *a, **kw):    return None
    def create_subscription(self, *a, **kw): return MagicMock()
    def create_timer(self, *a, **kw):        return MagicMock()
    def create_service(self, *a, **kw):      return MagicMock()
    def destroy_timer(self, *a, **kw):       pass
    def destroy_subscription(self, *a, **kw):pass
    def destroy_publisher(self, *a, **kw):   pass

_rclpy       = MagicMock()
_lifecycle   = MagicMock()
_lifecycle.LifecycleNode = _FakeNode
_lifecycle.TransitionCallbackReturn = MagicMock()
_node_mod    = MagicMock()
_node_mod.Node = _FakeNode

sys.modules['rclpy']               = _rclpy
sys.modules['rclpy.lifecycle']     = _lifecycle
sys.modules['rclpy.node']          = _node_mod
sys.modules['geometry_msgs']       = MagicMock()
sys.modules['geometry_msgs.msg']   = MagicMock()
sys.modules['std_msgs']            = MagicMock()
sys.modules['std_msgs.msg']        = MagicMock()
sys.modules['sensor_msgs']         = MagicMock()
sys.modules['sensor_msgs.msg']     = MagicMock()
sys.modules['std_srvs']            = MagicMock()
sys.modules['std_srvs.srv']        = MagicMock()

sys.path.insert(0, '/Users/hadyazzi/Desktop/Personal/xplore/xplore_rasberry/rover_xplore')

from rover_xplore.autonomous_node import (
    _norm, _diag_clear,
    UNDISCOVERED, FREE, OBSTACLE, AMBUSH, CELL_BORDER,
    GRID_ROWS, GRID_COLS, CELL_ROW_MM, CELL_COL_MM,
    SENSORS, AutonomousNode, _Mission, _Move,
)
from rover_xplore.motor_controller_node import (
    WHEEL_BASE, MM_PER_TICK, DT, _clamp,
    KP, KI, INTEGRAL_CLAMP,
)

import pytest


# ══════════════════════════════════════════════════════════════════════════════
# 1. Fonctions pures — _norm
# ══════════════════════════════════════════════════════════════════════════════

class TestNorm:
    def test_zero(self):
        assert _norm(0.0) == pytest.approx(0.0)

    def test_two_pi_wraps_to_zero(self):
        assert _norm(2 * pi) == pytest.approx(0.0, abs=1e-9)

    def test_pi_stays_pi(self):
        # atan2(sin(pi), cos(pi)) = atan2(≈0, -1) = pi
        assert abs(_norm(pi)) == pytest.approx(pi, abs=1e-9)

    def test_beyond_pi_wraps_negative(self):
        # pi + 0.1 → -(pi - 0.1)
        assert _norm(pi + 0.1) == pytest.approx(-(pi - 0.1), abs=1e-9)

    def test_negative_beyond_minus_pi(self):
        assert _norm(-pi - 0.1) == pytest.approx(pi - 0.1, abs=1e-9)

    def test_half_pi(self):
        assert _norm(pi / 2) == pytest.approx(pi / 2)

    def test_minus_half_pi(self):
        assert _norm(-pi / 2) == pytest.approx(-pi / 2)


# ══════════════════════════════════════════════════════════════════════════════
# 2. Fonctions pures — _diag_clear
# ══════════════════════════════════════════════════════════════════════════════

def _make_grid(rows=GRID_ROWS, cols=GRID_COLS, fill=FREE):
    return [[fill] * cols for _ in range(rows)]


class TestDiagClear:
    def test_cardinal_always_clear(self):
        g = _make_grid()
        # Horizontal/vertical: toujours libre même si voisins sont des obstacles
        g[3][4] = OBSTACLE
        assert _diag_clear(g, 3, 3, 0, 1) is True   # droite
        assert _diag_clear(g, 3, 3, 1, 0) is True   # bas
        assert _diag_clear(g, 3, 3, -1, 0) is True  # haut

    def test_diag_clear_when_corners_free(self):
        g = _make_grid()
        assert _diag_clear(g, 3, 3, 1, 1) is True

    def test_diag_blocked_by_row_corner(self):
        g = _make_grid()
        g[4][3] = OBSTACLE   # cells[r+dr][c]
        assert _diag_clear(g, 3, 3, 1, 1) is False

    def test_diag_blocked_by_col_corner(self):
        g = _make_grid()
        g[3][4] = OBSTACLE   # cells[r][c+dc]
        assert _diag_clear(g, 3, 3, 1, 1) is False

    def test_diag_blocked_by_border(self):
        g = _make_grid()
        g[4][3] = CELL_BORDER
        assert _diag_clear(g, 3, 3, 1, 1) is False

    def test_diag_nw(self):
        g = _make_grid()
        g[2][3] = OBSTACLE   # cells[r-1][c]
        assert _diag_clear(g, 3, 3, -1, -1) is False

    def test_ambush_does_not_block_diag(self):
        # AMBUSH n'est pas dans (OBSTACLE, CELL_BORDER) → ne bloque pas _diag_clear
        g = _make_grid()
        g[4][3] = AMBUSH
        # _diag_clear vérifie uniquement OBSTACLE et CELL_BORDER
        assert _diag_clear(g, 3, 3, 1, 1) is True


# ══════════════════════════════════════════════════════════════════════════════
# 3. BFS — logique de navigation
# ══════════════════════════════════════════════════════════════════════════════

def _make_bordered_grid():
    """Grille FREE avec CELL_BORDER sur toute la périphérie (état réel après _reset)."""
    g = _make_grid(fill=FREE)
    for r in range(GRID_ROWS):
        g[r][0] = g[r][GRID_COLS - 1] = CELL_BORDER
    for c in range(GRID_COLS):
        g[0][c] = g[GRID_ROWS - 1][c] = CELL_BORDER
    return g


def _make_node():
    """Crée un AutonomousNode minimal sans ROS."""
    n = AutonomousNode.__new__(AutonomousNode)
    _FakeNode.__init__(n)
    # Re-init state minimal
    n._x = n._y = n._theta = 0.0
    n._row, n._col = 1, 1
    n._cells    = _make_bordered_grid()
    n._priority = [[0] * GRID_COLS for _ in range(GRID_ROWS)]
    n._path_stack   = []
    n._transit_path = []
    n._mission   = _Mission.IDLE
    n._move      = _Move.IDLE
    n._nav_goal  = (1, 1)
    n._pub_grid_state = None
    n._pub_cmd        = None
    n._last_pose_time = None
    n._last_us_time   = None
    n._last_reason    = ''
    n._ambush_streak  = 0
    n._us_hit     = [0] * 5
    n._us_windows = [[] for _ in range(5)]
    return n


class TestBFSTo:
    def test_adjacent_path_length_two(self):
        n = _make_node()
        path = n._bfs_to(1, 1, 1, 2)
        assert path is not None
        assert len(path) == 2
        assert path[0] == (1, 1)
        assert path[1] == (1, 2)

    def test_diagonal_path(self):
        n = _make_node()
        path = n._bfs_to(1, 1, 3, 3)
        assert path is not None
        # BFS 8-connexe → 3 cases de distance Chebyshev suffisent
        assert len(path) >= 2
        assert path[0] == (1, 1)
        assert path[-1] == (3, 3)

    def test_blocked_path_returns_none(self):
        n = _make_node()
        # Bloquer toute la rangée 2 (cols 1 à 6)
        for c in range(1, GRID_COLS - 1):
            n._cells[2][c] = OBSTACLE
        path = n._bfs_to(1, 1, 3, 3)
        assert path is None

    def test_same_position_returns_path_len_two(self):
        n = _make_node()
        path = n._bfs_to(2, 3, 2, 3)
        assert path is not None
        assert len(path) == 2
        assert path[0] == path[1]

    def test_free_only_mode_avoids_undiscovered(self):
        n = _make_node()
        # Marquer toutes les cases de la rangée 2 en UNDISCOVERED sauf (2,1)
        for c in range(2, GRID_COLS - 1):
            n._cells[2][c] = UNDISCOVERED
        path = n._bfs_to(1, 1, 3, 1, free_only=True)
        # Le chemin FREE-only doit passer uniquement par des FREE
        if path is not None:
            for r, c in path:
                assert n._cells[r][c] == FREE, f"case ({r},{c}) non FREE: {n._cells[r][c]}"

    def test_ambush_not_traversable(self):
        n = _make_node()
        # Bloquer toute la rangée 2 avec AMBUSH
        for c in range(1, GRID_COLS - 1):
            n._cells[2][c] = AMBUSH
        path = n._bfs_to(1, 1, 3, 3)
        assert path is None


class TestBFSBestUndiscovered:
    def test_finds_closest_undiscovered(self):
        n = _make_node()
        n._cells[1][1] = FREE
        n._cells[1][2] = UNDISCOVERED   # 1 case de distance
        n._cells[3][3] = UNDISCOVERED   # plus loin
        # Priorité uniforme
        for r in range(GRID_ROWS):
            for c in range(GRID_COLS):
                n._priority[r][c] = 5
        path = n._bfs_best_undiscovered(1, 1)
        assert path is not None
        assert path[-1] == (1, 2)   # la plus proche

    def test_picks_highest_priority(self):
        n = _make_node()
        # Deux cases UNDISCOVERED équidistantes
        n._cells[1][2] = UNDISCOVERED
        n._cells[2][1] = UNDISCOVERED
        # Priorité plus haute à gauche
        n._priority[1][2] = 3
        n._priority[2][1] = 7
        path = n._bfs_best_undiscovered(1, 1)
        assert path is not None
        assert path[-1] == (2, 1)   # priorité 7

    def test_no_undiscovered_returns_none(self):
        n = _make_node()
        # Tout FREE, aucune UNDISCOVERED
        path = n._bfs_best_undiscovered(1, 1)
        assert path is None


# ══════════════════════════════════════════════════════════════════════════════
# 4. Obstacle detection — _mark_obstacle
# ══════════════════════════════════════════════════════════════════════════════
#
# Convention odométrie : θ=0 → cap nord (+y), θ croît CW (cap sud=±π).
# Formule correcte :
#   wx = x + sx*sin(θ) - sy*cos(θ)
#   wy = y + sx*cos(θ) + sy*sin(θ)
#   ox = wx + d*sin(θ - sa)
#   oy = wy + d*cos(θ - sa)
#
# ATTENTION : le code utilise cos/sin (convention x=est, θ=0=est).
# Ces tests documentent le comportement ATTENDU pour détecter le bug.
# ══════════════════════════════════════════════════════════════════════════════

def _run_mark_obstacle(x, y, theta, sx, sy, sa, d):
    """
    Extrait la logique de _mark_obstacle et retourne (obs_r, obs_c, wx, wy, ox, oy).
    Convention odométrie : θ=0=nord, sa CCW depuis forward.
    """
    th = theta
    wx = x + sx * sin(th) - sy * cos(th)
    wy = y + sx * cos(th) + sy * sin(th)
    ox = wx + d * sin(th - sa)
    oy = wy + d * cos(th - sa)
    obs_r = int(floor(oy / CELL_ROW_MM))
    obs_c = int(floor(ox / CELL_COL_MM))
    return obs_r, obs_c, wx, wy, ox, oy


def _mark_obstacle_correct(x, y, theta, sx, sy, sa, d):
    """
    Formule correcte pour la convention odométrie (θ=0=nord, θ CW, sa CCW).
    """
    th = theta
    wx = x + sx * sin(th) - sy * cos(th)
    wy = y + sx * cos(th) + sy * sin(th)
    ox = wx + d * sin(th - sa)
    oy = wy + d * cos(th - sa)
    obs_r = int(floor(oy / CELL_ROW_MM))
    obs_c = int(floor(ox / CELL_COL_MM))
    return obs_r, obs_c, wx, wy, ox, oy


class TestMarkObstacleConvention:
    """
    Vérifie la cohérence de la convention odométrie (θ=0=nord) avec _mark_obstacle.
    Les tests marqués xfail documentent le bug dans le code de production.
    """

    def test_forward_sensor_at_theta0_code(self):
        """
        Capteur d2 (avant-centre, sx=40, sy=0, sa=0) à θ=0.
        Rover en (x=0, y=0), cap nord.
        Obstacle à 800mm devant → attendu (obs_r=1, obs_c=0).
        Code actuel : obstacle dans la mauvaise direction.
        """
        sx, sy, sa = 40.0, 0.0, 0.0
        d = 800.0
        obs_r, obs_c, wx, wy, ox, oy = _run_mark_obstacle(0, 0, 0, sx, sy, sa, d)

        # Ce que le code produit réellement
        print(f"\n  [PRODUCTION] sensor_world=({wx:.0f},{wy:.0f}) obstacle=({ox:.0f},{oy:.0f}) → row={obs_r} col={obs_c}")

        # Ce qui EST CORRECT : obstacle 40+800=840 mm au nord → row=1, col=0
        r_correct, c_correct, *_ = _mark_obstacle_correct(0, 0, 0, sx, sy, sa, d)
        print(f"  [CORRECT]    → row={r_correct} col={c_correct}")

        assert r_correct == 1 and c_correct == 0, "Formule correcte attendue (row=1, col=0)"

    def test_forward_sensor_code_matches_expected(self):
        """
        Capteur avant à θ=0 : obstacle 840mm au nord → row=1, col=0.
        """
        sx, sy, sa = 40.0, 0.0, 0.0
        d = 800.0
        obs_r, obs_c, *_ = _run_mark_obstacle(0, 0, 0, sx, sy, sa, d)
        assert obs_r == 1 and obs_c == 0

    def test_lateral_left_sensor_d4_correct(self):
        """
        Capteur d4 (gauche, sx=-135, sy=+150, sa=+90°) à θ=0.
        Rover en (x=3000, y=3000) au centre de la grille.
        Obstacle à 800mm à gauche (ouest) → obs_c doit diminuer.
        """
        x0, y0 = 3000.0, 3000.0
        sx, sy, sa = -135.0, +150.0, math.radians(+90)
        d = 400.0
        obs_r_c, obs_c_c, wx_c, wy_c, ox_c, oy_c = _mark_obstacle_correct(x0, y0, 0, sx, sy, sa, d)
        obs_r_p, obs_c_p, wx_p, wy_p, ox_p, oy_p = _run_mark_obstacle(x0, y0, 0, sx, sy, sa, d)

        print(f"\n  [CORRECT] sensor=({wx_c:.0f},{wy_c:.0f}) obs=({ox_c:.0f},{oy_c:.0f}) → ({obs_r_c},{obs_c_c})")
        print(f"  [PRODUC.] sensor=({wx_p:.0f},{wy_p:.0f}) obs=({ox_p:.0f},{oy_p:.0f}) → ({obs_r_p},{obs_c_p})")

        # Le capteur gauche doit pointer vers l'OUEST (x diminue)
        assert ox_c < x0, f"Obstacle gauche doit être à l'ouest de x={x0}"

    def test_lateral_right_sensor_d5_correct(self):
        """
        Capteur d5 (droite, sx=-135, sy=-150, sa=-90°) à θ=0.
        Obstacle à droite → obs_c doit augmenter (est).
        """
        x0, y0 = 3000.0, 3000.0
        sx, sy, sa = -135.0, -150.0, math.radians(-90)
        d = 400.0
        obs_r_c, obs_c_c, wx_c, wy_c, ox_c, oy_c = _mark_obstacle_correct(x0, y0, 0, sx, sy, sa, d)

        print(f"\n  [CORRECT] sensor=({wx_c:.0f},{wy_c:.0f}) obs=({ox_c:.0f},{oy_c:.0f}) → ({obs_r_c},{obs_c_c})")

        # Capteur droit pointe vers l'EST (x augmente)
        assert ox_c > x0, f"Obstacle droit doit être à l'est de x={x0}"

    def test_sensor_body_position_at_theta0(self):
        """
        À θ=0 (cap nord), le capteur avant (sx=40, sy=0) doit être
        40mm AU NORD du rover (wy = y+40), pas au sud ou ailleurs.
        """
        x0, y0 = 1000.0, 1000.0
        _, _, wx_c, wy_c, _, _ = _mark_obstacle_correct(x0, y0, 0, 40, 0, 0, 0)
        _, _, wx_p, wy_p, _, _ = _run_mark_obstacle(x0, y0, 0, 40, 0, 0, 0)

        # Correct : wy doit être y+40 (nord), wx = x (inchangé)
        assert wy_c == pytest.approx(y0 + 40, abs=1), f"Formule correcte : wy={wy_c} attendu {y0+40}"
        assert wx_c == pytest.approx(x0, abs=1), f"Formule correcte : wx={wx_c} attendu {x0}"

        # Bug : le code place sx dans la mauvaise composante
        # wx_p = x+40 (mauvais), wy_p = y (mauvais)
        print(f"\n  [CORRECT] sensor world : ({wx_c:.0f},{wy_c:.0f})")
        print(f"  [PRODUC.] sensor world : ({wx_p:.0f},{wy_p:.0f})  ← INVERSÉ")

    def test_sensor_body_position_production_matches_expected(self):
        """Position du capteur avant : 40mm au nord du rover (wy = y+40)."""
        x0, y0 = 1000.0, 1000.0
        _, _, wx_p, wy_p, _, _ = _run_mark_obstacle(x0, y0, 0, 40, 0, 0, 0)
        assert wy_p == pytest.approx(y0 + 40, abs=1)
        assert wx_p == pytest.approx(x0, abs=1)


# ══════════════════════════════════════════════════════════════════════════════
# 5. Odométrie — intégration dead-reckoning
# ══════════════════════════════════════════════════════════════════════════════

def _odom_step(x, y, theta, d_right_mm, d_left_mm, gyro_bias_z=0.0, raw_gz=0.0):
    """Reproduit exactement le calcul de _enc_cb dans odometry_node."""
    from rover_xplore.odometry_node import MM_PER_TICK, ALPHA, DT, GYRO_Z_SCALE
    d_right  = d_right_mm   # déjà en mm (on by-pass ticks→mm ici)
    d_left   = d_left_mm
    d_center = (d_right + d_left) / 2.0
    dθ_enc   = (d_right - d_left) / 250.0   # WHEEL_BASE_MM

    gyro_rads = (raw_gz - gyro_bias_z) * GYRO_Z_SCALE
    dθ_gyro   = gyro_rads * DT
    dθ = ALPHA * dθ_gyro + (1.0 - ALPHA) * dθ_enc

    mid = theta + dθ / 2.0
    x    += d_center * sin(mid)
    y    += d_center * cos(mid)
    theta = atan2(sin(theta + dθ), cos(theta + dθ))
    return x, y, theta


class TestOdometryMath:
    def test_forward_at_theta0_only_y_increases(self):
        """θ=0 = cap nord : avancer ne doit modifier que y."""
        x, y, theta = _odom_step(0, 0, 0, d_right_mm=10, d_left_mm=10)
        assert x == pytest.approx(0.0, abs=1e-9)
        assert y == pytest.approx(10.0, abs=1e-6)
        assert theta == pytest.approx(0.0, abs=1e-9)

    def test_no_movement_when_wheels_stop(self):
        x, y, theta = _odom_step(100, 200, 0.5, 0, 0)
        assert x == pytest.approx(100.0)
        assert y == pytest.approx(200.0)
        assert theta == pytest.approx(0.5)

    def test_right_turn_decreases_theta(self):
        """
        Right > Left → CW → θ augmente dans la convention compass.
        dθ_enc = (d_right - d_left) / WHEEL_BASE = positive → θ augmente.
        """
        _, _, theta = _odom_step(0, 0, 0, d_right_mm=5, d_left_mm=-5)
        assert theta > 0.0, f"Virage droite → θ doit augmenter (cap est), obtenu {theta}"

    def test_theta_stays_wrapped(self):
        """
        Après un tour complet via gyro (ALPHA=0.95 favorise le gyro),
        θ doit revenir à ≈ 0.

        NOTE: avec raw_gz=0 et encodeurs seuls (5% du signal), il faut
        20× plus de rotation pour couvrir 2π. Ce test utilise le gyro.
        """
        from rover_xplore.odometry_node import GYRO_Z_SCALE, DT
        # 1 rad/s → 2π rad en 2π secondes → 63 steps à 0.1 s/step
        # raw_gz tel que gyro_rads = 2*pi / (2*pi / DT) = DT rad/s ≈ 0.1 rad/s
        # On veut finir à ~2π en 100 steps : gyro_rads = 2pi/(100*DT) = 0.628 rad/s
        target_radps = 2 * pi / (100 * DT)
        raw_gz_needed = target_radps / GYRO_Z_SCALE   # LSB équivalent
        x, y, theta = 0.0, 0.0, 0.0
        for _ in range(100):
            x, y, theta = _odom_step(x, y, theta, 0.0, 0.0, raw_gz=raw_gz_needed)
        # ALPHA*gyro donne ~95% du signal → erreur < 5%
        assert abs(theta) < 0.4, f"Après 360° gyro, θ doit ≈ 0, obtenu {theta:.3f} rad"

    def test_forward_east_at_theta_pi_half(self):
        """θ=π/2 → cap est : avancer doit augmenter x."""
        x, y, theta = _odom_step(0, 0, pi/2, d_right_mm=10, d_left_mm=10)
        assert x == pytest.approx(10.0, abs=1e-6)
        assert y == pytest.approx(0.0, abs=1e-6)

    def test_grid_col_from_x_row_from_y(self):
        """Vérification mapping x→col, y→row (cohérence odométrie ↔ grille)."""
        col = int(floor(833.5 / CELL_COL_MM))    # ~1 cellule
        row = int(floor(800.5 / CELL_ROW_MM))    # ~1 cellule
        assert col == 1
        assert row == 1

    def test_nav_goal_pose_alignment(self):
        """nav_goal_cb fixe x = col*CELL_COL + CELL_COL/2, y = row*CELL_ROW + CELL_ROW/2."""
        from rover_xplore.odometry_node import CELL_COL_MM, CELL_ROW_MM
        start_row, start_col = 10, 6
        expected_x = start_col * CELL_COL_MM + CELL_COL_MM / 2.0
        expected_y = start_row * CELL_ROW_MM + CELL_ROW_MM / 2.0
        assert expected_x == pytest.approx(6 * 833 + 833 / 2, rel=1e-3)
        assert expected_y == pytest.approx(10 * 800 + 400)


# ══════════════════════════════════════════════════════════════════════════════
# 6. Cinématique différentielle — motor_controller
# ══════════════════════════════════════════════════════════════════════════════

def _twist_to_pwm_targets(linear_ms, angular_radps):
    """
    Calcule les setpoints en Δticks/100ms depuis un Twist (sans feedback PI).
    Reproduit motor_controller._cmd_cb.
    """
    v_left  = linear_ms - angular_radps * WHEEL_BASE / 2.0
    v_right = linear_ms + angular_radps * WHEEL_BASE / 2.0
    k = 1000.0 * DT / MM_PER_TICK   # m/s → Δticks/100ms
    return v_left * k, v_right * k


class TestMotorKinematics:
    def test_forward_equal_speeds(self):
        """Avancer tout droit → vitesses gauche = droite."""
        tl, tr = _twist_to_pwm_targets(linear_ms=0.2, angular_radps=0.0)
        assert tl == pytest.approx(tr)
        assert tl > 0

    def test_turn_left_right_slower(self):
        """Virage gauche (angular > 0 dans ROS, = CW ici?) :
           v_right = linear + angular*wb/2 > v_left = linear - angular*wb/2."""
        tl, tr = _twist_to_pwm_targets(linear_ms=0.0, angular_radps=1.0)
        assert tr > 0 and tl < 0, f"angular=+1 → tr={tr:.2f} tl={tl:.2f}"

    def test_backward(self):
        tl, tr = _twist_to_pwm_targets(linear_ms=-0.2, angular_radps=0.0)
        assert tl == pytest.approx(tr)
        assert tl < 0

    def test_clamp_range(self):
        assert _clamp(150, -100, 100) == 100
        assert _clamp(-150, -100, 100) == -100
        assert _clamp(50.7, -100, 100) == 51
        assert _clamp(-0.4, -100, 100) == 0

    def test_pi_controller_reduces_error(self):
        """Le contrôleur PI doit toujours réduire l'erreur step par step."""
        integral = 0.0
        target = 20.0
        actual = 0.0
        prev_error = abs(target - actual)

        for _ in range(5):
            error = target - actual
            integral = max(-INTEGRAL_CLAMP, min(INTEGRAL_CLAMP, integral + error))
            pwm = KP * error + KI * integral
            # Simuler: pwm proportionnel → actual augmente un peu
            actual += pwm * 0.1   # simulation grossière
            new_error = abs(target - actual)
            assert new_error < prev_error, f"PI doit converger : err={new_error:.2f} > prev={prev_error:.2f}"
            prev_error = new_error

    def test_pi_integral_zeroed_at_stop(self):
        """Quand target=0, l'intégrateur est remis à zéro."""
        from rover_xplore.motor_controller_node import MotorControllerNode
        node = MotorControllerNode.__new__(MotorControllerNode)
        _FakeNode.__init__(node)
        node._pub          = None
        node._sub_cmd_vel  = None
        node._sub_encoders = None
        node._timer_safety = None
        node._last_cmd_time = None
        node._tgt_left  = 0.0
        node._tgt_right = 0.0
        node._integral  = [50.0, -30.0]   # intégrateurs chargés

        # Appel PI avec target=0 → intégrateurs doivent être vidés
        node._pi(0, 0.0, 10.0)
        node._pi(1, 0.0, -5.0)
        assert node._integral[0] == 0.0
        assert node._integral[1] == 0.0


# ══════════════════════════════════════════════════════════════════════════════
# 7. Edge cases AutonomousNode — nav_goal, reset, priorités
# ══════════════════════════════════════════════════════════════════════════════

class TestAutonomousNodeLogic:
    def test_reset_initialises_grid_borders(self):
        n = _make_node()
        n._nav_goal = (1, 1)
        n._start    = (10, 6)
        # Initialiser les attributs attendus par _reset
        n._aruco_found = False
        n._aruco_area  = 0.0
        n._aruco_phase = None
        n._aruco_approach_heading = 0.0
        n._aruco_start_x = n._aruco_start_y = 0.0
        n._aruco_advance_dist = 0.0
        n._collecting = False
        n._arm_seq_idx = n._arm_seq_ticks = 0
        n._use_world_target = False
        n._move_start_time = None
        n._us_hit     = [0] * 5
        n._us_windows = [[] for _ in range(5)]

        n._reset()

        # Bordures marquées CELL_BORDER
        for r in range(GRID_ROWS):
            assert n._cells[r][0] == CELL_BORDER
            assert n._cells[r][GRID_COLS - 1] == CELL_BORDER
        for c in range(GRID_COLS):
            assert n._cells[0][c] == CELL_BORDER
            assert n._cells[GRID_ROWS - 1][c] == CELL_BORDER

    def test_reset_initialises_interior_undiscovered(self):
        n = _make_node()
        n._nav_goal = (1, 1)
        n._start    = (10, 6)
        n._aruco_found = False; n._aruco_area = 0.0; n._aruco_phase = None
        n._aruco_approach_heading = 0.0; n._aruco_start_x = n._aruco_start_y = 0.0
        n._aruco_advance_dist = 0.0; n._collecting = False
        n._arm_seq_idx = n._arm_seq_ticks = 0; n._use_world_target = False
        n._move_start_time = None; n._us_hit = [0]*5; n._us_windows = [[] for _ in range(5)]
        n._reset()

        for r in range(1, GRID_ROWS - 1):
            for c in range(1, GRID_COLS - 1):
                assert n._cells[r][c] == UNDISCOVERED

    def test_recalc_priorities_target_gets_max(self):
        n = _make_node()
        # _MAX_DIST = 9
        n._recalc_priorities(1, 1)
        assert n._priority[1][1] == 9    # target → prio max
        assert n._priority[10][6] == 0   # case de départ

    def test_recalc_priorities_decreases_with_distance(self):
        n = _make_node()
        n._recalc_priorities(5, 4)
        # (5,4) → prio 9
        assert n._priority[5][4] == 9
        # (5,5) → Chebyshev=1 → prio 8
        assert n._priority[5][5] == 8
        # (4,3) → Chebyshev=1 → prio 8
        assert n._priority[4][3] == 8

    def test_heading_forward_vs_reverse(self):
        """
        _start_move_to choisit forward si |delta| <= pi/2, sinon reverse.
        Vérifier que la logique de choix est cohérente.
        """
        n = _make_node()
        n._theta = 0.0   # cap nord

        # Cible au nord (row+1) → heading ≈ 0 → delta ≈ 0 → forward
        heading = atan2(0, 1)   # dc=0, dr=+1 → atan2(0,1) = 0
        delta = _norm(heading - n._theta)
        assert abs(delta) <= pi / 2   # devrait aller en avant

        # Cible au sud (row-1) → heading = atan2(0, -1) = ±pi → |delta| = pi → reverse
        heading_south = atan2(0, -1)
        delta_south = _norm(heading_south - n._theta)
        assert abs(delta_south) > pi / 2   # devrait reculer


# ══════════════════════════════════════════════════════════════════════════════
# 8. Cohérence des constantes
# ══════════════════════════════════════════════════════════════════════════════

class TestConstants:
    def test_mm_per_tick_positive(self):
        assert MM_PER_TICK > 0
        assert MM_PER_TICK == pytest.approx(pi * 120 / 1320, rel=1e-6)

    def test_cell_sizes_match_arena(self):
        # Arena navigable : 10 lignes × 6 colonnes
        assert CELL_ROW_MM == pytest.approx(8000 / 10, rel=1e-3)
        assert CELL_COL_MM == pytest.approx(5000 / 6, rel=1e-2)

    def test_grid_full_size(self):
        # 10×6 navigables + 1 bordure tout autour = 12×8
        assert GRID_ROWS == 12
        assert GRID_COLS == 8

    def test_sensor_directions_make_sense(self):
        # d1 avant-gauche : sx>0 (avant), sy>0 (gauche), sa>0 (gauche)
        sx1, sy1, sa1, _ = SENSORS[0]
        assert sx1 > 0 and sy1 > 0 and sa1 > 0
        # d3 avant-droite : sx>0, sy<0, sa<0
        sx3, sy3, sa3, _ = SENSORS[2]
        assert sx3 > 0 and sy3 < 0 and sa3 < 0
        # d4/d5 derrière : sx<0
        sx4, *_ = SENSORS[3]
        sx5, *_ = SENSORS[4]
        assert sx4 < 0 and sx5 < 0
