# ══════════════════════════════════════════════════════════════════════════════
# serial_bridge_node.py — Raspberry Pi
#
# RÔLE : pont binaire exclusif entre ROS2 et l'Arduino via UART/USB.
#         Seul propriétaire du port série — aucun autre node ne touche au port.
#
# ENVOI (Pi → Micro) à 10 Hz — struct 18 octets little-endian :
#   uint16 servo_1, servo_2, servo_3, servo_4   (-100..100 encodé en uint16)
#   int16  motor1..4                             (-255..255)
#   int16  stepper                               (-100..100)
#
# RÉCEPTION (Micro → Pi) à chaque tick — struct 30 octets :
#   uint16 accel_x/y/z, gyro_x/y/z             (IMU brut)
#   int16  motor1..4                             (vitesses encodeurs roues)
#   uint16 distance_1..5                         (US en cm)
#
# TOPICS ÉCOUTÉS :
#   /rover/motor_cmd     Int32MultiArray [m1, m2, m3, m4]  (-255..255)
#   /rover/arm_serial_cmd Int32MultiArray [s1,s2,s3,s4,stepper] (-100..100)
#   /rover/mode          String — force zeros si 'idle'
#
# TOPICS PUBLIÉS :
#   /ultrasonic          Float32MultiArray [d1..d5] en cm
#   /imu/raw             sensor_msgs/Imu   (accél + gyro bruts)
#   /wheel_encoders      Int32MultiArray   [m1, m2, m3, m4]
# ══════════════════════════════════════════════════════════════════════════════

import struct

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32MultiArray, Int32MultiArray, String

import serial

# Format des structs binaires — little-endian
_FMT_SEND = '<4H5h'   # 4×uint16 (servos) + 4×int16 (motors) + 1×int16 (stepper) = 18 octets
_FMT_RECV = '<6H4h5H' # 6×uint16 (imu) + 4×int16 (enc) + 5×uint16 (US) = 30 octets
_SIZE_SEND = struct.calcsize(_FMT_SEND)  # 18
_SIZE_RECV = struct.calcsize(_FMT_RECV)  # 30


def _clamp(v, lo, hi):
    return max(lo, min(hi, v))


def _to_uint16(signed_val):
    """Encode une valeur signée [-100..100] en uint16 two's complement."""
    return int(signed_val) & 0xFFFF


class SerialBridgeNode(Node):

    SERIAL_PORT = '/dev/ttyUSB0'
    BAUD_RATE   = 115200

    def __init__(self):
        super().__init__('serial_bridge_node')

        # État courant de la commande — mis à jour par les callbacks, envoyé au timer
        self._motors  = [0, 0, 0, 0]   # [FR, FL, BR, BL], int16, -255..255
        self._servos  = [0, 0, 0, 0]   # [s1, s2, s3, s4], int16 encodé uint16, -100..100
        self._stepper = 0              # int16, -100..100
        self._idle    = True           # True → force zeros dans envoi

        self._ser = None
        self._connect_serial()

        # Subscriptions
        self.create_subscription(Int32MultiArray, '/rover/motor_cmd',      self._motor_cb,  10)
        self.create_subscription(Int32MultiArray, '/rover/arm_serial_cmd', self._arm_cb,    10)
        self.create_subscription(String,          '/rover/mode',           self._mode_cb,   10)

        # Publishers
        self._pub_us  = self.create_publisher(Float32MultiArray, '/ultrasonic',     10)
        self._pub_imu = self.create_publisher(Imu,               '/imu/raw',        10)
        self._pub_enc = self.create_publisher(Int32MultiArray,   '/wheel_encoders', 10)

        # Timer 10 Hz : envoi + tentative de lecture
        self.create_timer(0.1, self._tick)

        self.get_logger().info('serial_bridge_node démarré')

    # ── Connexion série ───────────────────────────────────────────────────────

    def _connect_serial(self):
        try:
            self._ser = serial.Serial(self.SERIAL_PORT, self.BAUD_RATE, timeout=0.05)
            self.get_logger().info(f'Serial ouvert : {self.SERIAL_PORT} @ {self.BAUD_RATE}')
        except serial.SerialException as e:
            self._ser = None
            self.get_logger().warn(f'Serial indisponible ({e}) — mode log uniquement')

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _mode_cb(self, msg: String):
        self._idle = (msg.data == 'idle')
        if self._idle:
            self._motors  = [0, 0, 0, 0]
            self._servos  = [0, 0, 0, 0]
            self._stepper = 0

    def _motor_cb(self, msg: Int32MultiArray):
        if len(msg.data) >= 4:
            self._motors = [_clamp(int(v), -255, 255) for v in msg.data[:4]]

    def _arm_cb(self, msg: Int32MultiArray):
        # [servo1, servo2, servo3, servo4, stepper]
        if len(msg.data) >= 5:
            self._servos  = [_clamp(int(v), -100, 100) for v in msg.data[:4]]
            self._stepper = _clamp(int(msg.data[4]), -100, 100)

    # ── Tick 10 Hz ────────────────────────────────────────────────────────────

    def _tick(self):
        self._send_struct()
        self._recv_struct()

    # ── Envoi struct Pi → Micro ───────────────────────────────────────────────

    def _send_struct(self):
        if not self._ser or not self._ser.is_open:
            return

        motors  = self._motors  if not self._idle else [0, 0, 0, 0]
        servos  = self._servos  if not self._idle else [0, 0, 0, 0]
        stepper = self._stepper if not self._idle else 0

        payload = struct.pack(
            _FMT_SEND,
            _to_uint16(servos[0]),
            _to_uint16(servos[1]),
            _to_uint16(servos[2]),
            _to_uint16(servos[3]),
            motors[0], motors[1], motors[2], motors[3],
            stepper,
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
            if available < _SIZE_RECV:
                return

            # Si plusieurs frames ont été accumulées, prendre la plus récente
            if available > _SIZE_RECV:
                extra = available - (available % _SIZE_RECV)
                self._ser.read(extra - _SIZE_RECV)

            raw = self._ser.read(_SIZE_RECV)
            if len(raw) < _SIZE_RECV:
                return

            vals = struct.unpack(_FMT_RECV, raw)
            ax, ay, az, gx, gy, gz         = vals[0:6]
            m1, m2, m3, m4                 = vals[6:10]
            d1, d2, d3, d4, d5             = vals[10:15]

        except serial.SerialException as e:
            self.get_logger().error(f'Erreur lecture serial : {e}')
            self._ser = None
            return
        except struct.error as e:
            self.get_logger().warn(f'Struct corrompue : {e}')
            return

        # /ultrasonic
        us = Float32MultiArray()
        us.data = [float(d1), float(d2), float(d3), float(d4), float(d5)]
        self._pub_us.publish(us)

        # /imu/raw
        imu = Imu()
        imu.header.stamp            = self.get_clock().now().to_msg()
        imu.linear_acceleration.x   = float(ax)
        imu.linear_acceleration.y   = float(ay)
        imu.linear_acceleration.z   = float(az)
        imu.angular_velocity.x      = float(gx)
        imu.angular_velocity.y      = float(gy)
        imu.angular_velocity.z      = float(gz)
        self._pub_imu.publish(imu)

        # /wheel_encoders
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
        rclpy.shutdown()


if __name__ == '__main__':
    main()
