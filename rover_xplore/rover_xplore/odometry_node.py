# ══════════════════════════════════════════════════════════════════════════════
# odometry_node.py — Raspberry Pi  (plain Node, toujours actif)
#
# Dead-reckoning : encodeurs de roues + gyroscope MPU9250.
# Filtre complémentaire alpha=0.95 (gyro) / 0.05 (encodeurs).
#
# CALIBRATION GYRO au démarrage : 100 échantillons avec rover immobile.
# Pendant la calibration, aucune pose n'est publiée.
#
# TOPICS ÉCOUTÉS :
#   /wheel_encoders  Int32MultiArray  [FR, FL, BR, BL]  Δticks / 100ms
#   /imu/raw         sensor_msgs/Imu  angular_velocity.z (LSB int16 MPU9250)
#   /rover/nav_goal  Int32MultiArray  [start_row, start_col, target_row, target_col]
#
# TOPICS PUBLIÉS :
#   /rover/pose      Float32MultiArray  [x_mm, y_mm, theta_rad]
#   /rover/grid_pos  Int32MultiArray    [col, row]
#
# SERVICE :
#   /rover/reset_pose  std_srvs/Trigger  → remet (x, y, θ) à zéro
# ══════════════════════════════════════════════════════════════════════════════

from math import atan2, floor, pi, sin

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32MultiArray, Int32MultiArray
from std_srvs.srv import Trigger

# ── Grille ────────────────────────────────────────────────────────────────────
GRID_ROWS   = 12
GRID_COLS   = 8
CELL_ROW_MM = 800   # 8000 mm / 10 lignes navigables
CELL_COL_MM = 833   # 5000 mm / 6 colonnes navigables

# ── Physique (à calibrer) ─────────────────────────────────────────────────────
WHEEL_DIAMETER_MM = 120.0
WHEEL_BASE_MM     = 250.0
TICKS_PER_REV     = 1320                                # JGA25-370 30:1 — à calibrer
MM_PER_TICK       = (pi * WHEEL_DIAMETER_MM) / TICKS_PER_REV   # ≈ 0.285 mm/tick
ENC_SIGN          = -1                                  # encodeurs négatifs en avance → inverser

# ── Filtre complémentaire ─────────────────────────────────────────────────────
ALPHA        = 0.95                      # poids gyro
GYRO_Z_SCALE = pi / (180.0 * 131.0)    # MPU9250 ±250°/s → rad/s par LSB ≈ 1.33e-4
DT           = 0.1                       # s — période serial_bridge

CALIBRATION_SAMPLES = 100


class OdometryNode(Node):

    def __init__(self):
        super().__init__('odometry_node')

        self._x     = 0.0
        self._y     = 0.0
        self._theta = 0.0

        self._last_raw_gz  = 0.0
        self._gyro_samples = []
        self._gyro_bias_z  = 0.0
        self._calibrated   = False

        self._pub_pose = self.create_publisher(Float32MultiArray, '/rover/pose',     10)
        self._pub_grid = self.create_publisher(Int32MultiArray,   '/rover/grid_pos', 10)

        self.create_subscription(Int32MultiArray, '/wheel_encoders', self._enc_cb, 10)
        self.create_subscription(Imu,             '/imu/raw',        self._imu_cb, 10)
        self.create_subscription(Int32MultiArray, '/rover/nav_goal', self._nav_goal_cb, 10)

        self.create_service(Trigger, '/rover/reset_pose', self._reset_cb)

        self.get_logger().info(
            f'odometry_node démarré — calibration gyro '
            f'({CALIBRATION_SAMPLES} samples, rover immobile)…'
        )

    # ── IMU ───────────────────────────────────────────────────────────────────

    def _imu_cb(self, msg: Imu):
        raw_gz = msg.angular_velocity.z
        # Guard: ancienne version serial_bridge pouvait envoyer uint16 non signé
        if raw_gz > 32767.0:
            raw_gz -= 65536.0
        self._last_raw_gz = raw_gz

        if self._calibrated:
            return

        self._gyro_samples.append(raw_gz)
        if len(self._gyro_samples) >= CALIBRATION_SAMPLES:
            self._gyro_bias_z = sum(self._gyro_samples) / len(self._gyro_samples)
            self._calibrated  = True
            self.get_logger().info(
                f'Calibration gyro OK — biais z = {self._gyro_bias_z:.2f} LSB'
            )

    # ── Encodeurs ─────────────────────────────────────────────────────────────

    def _enc_cb(self, msg: Int32MultiArray):
        if len(msg.data) < 4 or not self._calibrated:
            return

        m1, m2, m3, m4 = [ENC_SIGN * v for v in msg.data[:4]]   # FR, FL, BR, BL (Δticks / 100ms)

        d_right  = ((m1 + m3) / 2.0) * MM_PER_TICK
        d_left   = ((m2 + m4) / 2.0) * MM_PER_TICK
        d_center = (d_right + d_left) / 2.0
        dθ_enc   = (d_right - d_left) / WHEEL_BASE_MM

        gyro_rads = (self._last_raw_gz - self._gyro_bias_z) * GYRO_Z_SCALE
        dθ_gyro   = gyro_rads * DT

        dθ = ALPHA * dθ_gyro + (1.0 - ALPHA) * dθ_enc

        # Intégration midpoint heading.
        # Convention grille : theta=0 pointe vers row+1 (axe y positif).
        mid = self._theta + dθ / 2.0
        self._x    += d_center * sin(mid)
        self._y    += d_center * cos(mid)
        self._theta = atan2(sin(self._theta + dθ), cos(self._theta + dθ))

        self._publish()

    # ── Navigation goal ──────────────────────────────────────────────────────

    def _nav_goal_cb(self, msg: Int32MultiArray):
        if len(msg.data) < 4:
            return

        start_row = int(msg.data[0])
        start_col = int(msg.data[1])
        if not (0 <= start_row < GRID_ROWS and 0 <= start_col < GRID_COLS):
            self.get_logger().warn(f'nav_goal start hors grille : ({start_row},{start_col})')
            return

        self._x = start_col * CELL_COL_MM + CELL_COL_MM / 2.0
        self._y = start_row * CELL_ROW_MM + CELL_ROW_MM / 2.0
        self._theta = 0.0
        self._publish()
        self.get_logger().info(
            f'Pose alignée sur départ nav_goal : row={start_row}, col={start_col}'
        )

    # ── Publication ───────────────────────────────────────────────────────────

    def _publish(self):
        pose = Float32MultiArray()
        pose.data = [float(self._x), float(self._y), float(self._theta)]
        self._pub_pose.publish(pose)

        col = max(0, min(GRID_COLS - 1, int(floor(self._x / CELL_COL_MM))))
        row = max(0, min(GRID_ROWS - 1, int(floor(self._y / CELL_ROW_MM))))
        grid = Int32MultiArray()
        grid.data = [col, row]
        self._pub_grid.publish(grid)

    # ── Service reset ──────────────────────────────────────────────────────────

    def _reset_cb(self, _req, response):
        self._x = self._y = self._theta = 0.0
        response.success = True
        response.message = 'Pose réinitialisée (0, 0, 0)'
        self.get_logger().info('reset_pose — pose remise à zéro')
        return response


def main(args=None):
    rclpy.init(args=args)
    node = OdometryNode()
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
