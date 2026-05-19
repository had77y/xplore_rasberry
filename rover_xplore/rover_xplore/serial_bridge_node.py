# ══════════════════════════════════════════════════════════════════════════════
# serial_bridge_node.py — Raspberry Pi
#
# RÔLE : pont binaire exclusif entre ROS2 et l'ESP32 via UART/USB.
#         Seul propriétaire du port série — aucun autre node ne touche au port.
#         Toujours actif, indépendant du mode.
#
# PROTOCOLE request-response : l'ESP32 n'envoie QUE quand il reçoit.
#   → Le Pi envoie une trame, l'ESP32 répond immédiatement.
#
# ENVOI (Pi → ESP32) à 10 Hz — b"rover" + struct 18 octets little-endian :
#   int16  servo_1, servo_2, servo_3, servo_4   (-100..100)
#   int16  motor1..4                             (-100..100)
#   int16  stepper                               (-1 / 0 / +1)
#
# RÉCEPTION (ESP32 → Pi) — b"rover" + struct 30 octets :
#   int16  accel_x/y/z, gyro_x/y/z             (IMU brut, signé)
#   int16  motor1..4                             (vitesses encodeurs roues)
#   uint16 distance_1..5                         (US en mm)
#
# SÉCURITÉ :
#   Header ASCII b"rover" en tête de chaque trame (Pi→ESP32 et ESP32→Pi).
#   Le récepteur cherche ce marqueur avant de décoder — alignement robuste
#   même après reset, bruit UART ou buffer overflow.
#   Si aucun motor_cmd reçu depuis MOTOR_TIMEOUT_S → moteurs forcés à zéro.
#   Si aucun arm_serial_cmd reçu depuis ARM_TIMEOUT_S → stepper forcé à zéro.
#
# TOPICS ÉCOUTÉS :
#   /rover/motor_cmd      Int32MultiArray [m1, m2, m3, m4]       (-100..100)
#   /rover/arm_serial_cmd Int32MultiArray [s1, s2, s3, s4, step] (step = -1/0/+1)
#
# TOPICS PUBLIÉS :
#   /ultrasonic     Float32MultiArray [d1..d5] en mm
#   /imu/raw        sensor_msgs/Imu   (accél + gyro bruts)
#   /wheel_encoders Int32MultiArray   [m1, m2, m3, m4]
# ══════════════════════════════════════════════════════════════════════════════

import struct

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32MultiArray, Int32MultiArray

import serial

_MAGIC     = b'rover'
_FMT_SEND  = '<9h'                          # 18 octets : 9 × int16
_FMT_RECV  = '<10h5H'                       # 30 octets : 10 × int16 + 5 × uint16
_SIZE_DATA = struct.calcsize(_FMT_RECV)     # 30

MOTOR_TIMEOUT_S = 1.0   # sécurité : zéro moteurs si plus de commande depuis 1s
ARM_TIMEOUT_S   = 1.0   # sécurité : zéro stepper si plus de commande depuis 1s


def _clamp(v: int, lo: int, hi: int) -> int:
    return max(lo, min(hi, v))


class SerialBridgeNode(Node):

    SERIAL_PORT = '/dev/ttyUSB0'
    BAUD_RATE   = 115200

    def __init__(self):
        super().__init__('serial_bridge_node')

        # État courant envoyé à l'Arduino à chaque tick
        self._motors  = [0, 0, 0, 0]
        self._servos  = [0, 0, 0, 0]
        self._stepper = 0

        # Horodatage de la dernière commande moteur (pour le timeout sécurité)
        self._last_motor_time = self.get_clock().now()
        self._last_arm_time   = self.get_clock().now()

        self._ser = None
        self._reconnect_ticks = 0
        self._rx_buffer = bytearray()
        self._connect_serial()

        self.create_subscription(Int32MultiArray, '/rover/motor_cmd',      self._motor_cb, 10)
        self.create_subscription(Int32MultiArray, '/rover/arm_serial_cmd', self._arm_cb,   10)

        self._pub_us  = self.create_publisher(Float32MultiArray, '/ultrasonic',     10)
        self._pub_imu = self.create_publisher(Imu,               '/imu/raw',        10)
        self._pub_enc = self.create_publisher(Int32MultiArray,   '/wheel_encoders', 10)

        self.create_timer(0.1, self._tick)

        self.get_logger().info('serial_bridge_node démarré')

    # ── Connexion série ───────────────────────────────────────────────────────

    def _connect_serial(self):
        try:
            self._ser = serial.Serial(self.SERIAL_PORT, self.BAUD_RATE, timeout=0.05)
            # Vider le buffer à l'ouverture pour démarrer aligné sur une trame propre
            self._ser.reset_input_buffer()
            self.get_logger().info(f'Serial ouvert : {self.SERIAL_PORT} @ {self.BAUD_RATE}')
        except serial.SerialException:
            self._ser = None

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _motor_cb(self, msg: Int32MultiArray):
        if len(msg.data) >= 4:
            self._motors = [_clamp(int(v), -100, 100) for v in msg.data[:4]]
            self._last_motor_time = self.get_clock().now()

    def _arm_cb(self, msg: Int32MultiArray):
        if len(msg.data) >= 5:
            self._servos  = [_clamp(int(v), -100, 100) for v in msg.data[:4]]
            self._stepper = _clamp(int(msg.data[4]), -1, 1)
            self._last_arm_time = self.get_clock().now()

    # ── Tick 10 Hz ────────────────────────────────────────────────────────────

    def _tick(self):
        if self._ser is None:
            self._try_reconnect()
            return
        self._apply_motor_timeout()
        self._apply_arm_timeout()
        self._send_struct()
        self._recv_struct()

    def _apply_motor_timeout(self):
        elapsed = (self.get_clock().now() - self._last_motor_time).nanoseconds / 1e9
        if elapsed > MOTOR_TIMEOUT_S and any(v != 0 for v in self._motors):
            self._motors = [0, 0, 0, 0]
            self.get_logger().warn('Timeout motor_cmd — moteurs forcés à zéro')

    def _apply_arm_timeout(self):
        elapsed = (self.get_clock().now() - self._last_arm_time).nanoseconds / 1e9
        if elapsed > ARM_TIMEOUT_S and self._stepper != 0:
            self._stepper = 0
            self.get_logger().warn('Timeout arm_serial_cmd — stepper forcé à zéro')

    def _try_reconnect(self):
        """Tente de rouvrir le port série après une déconnexion."""
        self._reconnect_ticks += 1
        if self._reconnect_ticks % 50 == 1:  # log toutes les 5s (50 ticks × 100ms)
            self.get_logger().warn('Serial indisponible — tentative de reconnexion...')
        self._connect_serial()

    # ── Envoi struct Pi → Micro ───────────────────────────────────────────────

    def _send_struct(self):
        if not self._ser or not self._ser.is_open:
            return

        payload = _MAGIC + struct.pack(
            _FMT_SEND,
            self._servos[0], self._servos[1], self._servos[2], self._servos[3],
            self._motors[0], self._motors[1], self._motors[2], self._motors[3],
            self._stepper,
        )

        try:
            self._ser.write(payload)
        except serial.SerialException as e:
            self.get_logger().error(f'Erreur envoi serial : {e}')
            self._ser = None

    # ── Lecture struct Micro → Pi ─────────────────────────────────────────────

    def _recv_struct(self):
        if not self._ser or not self._ser.is_open:
            return

        try:
            available = self._ser.in_waiting
            if available <= 0:
                return

            self._rx_buffer.extend(self._ser.read(available))

            # Cherche la dernière occurrence du magic (trame la plus récente).
            # Fonctionne même si le buffer contient plusieurs trames ou des
            # octets parasites — les trames incomplètes restent en attente.
            idx = self._rx_buffer.rfind(_MAGIC)
            if idx == -1:
                # Garde seulement une petite fenêtre au cas où le header arrive
                # coupé entre deux lectures.
                del self._rx_buffer[:-len(_MAGIC)]
                return

            data_start = idx + len(_MAGIC)
            if len(self._rx_buffer) - data_start < _SIZE_DATA:
                return  # trame incomplète après le magic

            frame_end = data_start + _SIZE_DATA
            raw  = self._rx_buffer[data_start:frame_end]
            del self._rx_buffer[:frame_end]
            vals = struct.unpack(_FMT_RECV, raw)

            ax, ay, az, gx, gy, gz = vals[0:6]
            m1, m2, m3, m4         = vals[6:10]
            d1, d2, d3, d4, d5     = vals[10:15]

        except serial.SerialException as e:
            self.get_logger().error(f'Erreur lecture serial : {e}')
            self._ser = None
            return
        except struct.error as e:
            self.get_logger().warn(f'Struct corrompue : {e}')
            return

        us = Float32MultiArray()
        us.data = [float(d1), float(d2), float(d3), float(d4), float(d5)]
        self._pub_us.publish(us)

        imu = Imu()
        imu.header.stamp          = self.get_clock().now().to_msg()
        imu.linear_acceleration.x = float(ax)
        imu.linear_acceleration.y = float(ay)
        imu.linear_acceleration.z = float(az)
        imu.angular_velocity.x    = float(gx)
        imu.angular_velocity.y    = float(gy)
        imu.angular_velocity.z    = float(gz)
        self._pub_imu.publish(imu)

        enc = Int32MultiArray()
        enc.data = [int(m1), int(m2), int(m3), int(m4)]
        self._pub_enc.publish(enc)

    # ── Nettoyage ─────────────────────────────────────────────────────────────

    def destroy_node(self):
        if self._ser and self._ser.is_open:
            self._ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SerialBridgeNode()
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
