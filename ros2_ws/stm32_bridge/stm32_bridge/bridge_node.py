#!/usr/bin/env python3
"""
stm32_bridge — uart bridge between ros2 and the stm32.

frame: [0xAA][0x55][cmd][len][payload... len bytes][chk]
chk = xor of cmd, len and every payload byte. multi-byte fields little-endian.

subscribes:
  /wheel_speeds  Float32MultiArray  FL FR RL RR, -1.0 .. +1.0  -> drive
  /gripper       Int32              0..100                     -> gripper
  /hopper        Int32              0 stop / 1 extend / 2 retract
  /arm           Float32MultiArray  [base, shoulder, elbow] deg (+ optional time_ms)

publishes:
  /wheel_encoders  Int32MultiArray  FL FR RL RR cumulative ticks

params:
  serial_port  (string)  default /dev/ttyTHS1
  baud_rate    (int)     default 115200
"""

import struct
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Int32MultiArray, Int32

try:
    import serial
except ImportError:
    raise SystemExit("pyserial not found — run: pip3 install pyserial")

# ── protocol (must match comms_protocol.h) ────────────────────────────────
HEADER      = bytes([0xAA, 0x55])
CMD_DRIVE   = 0x01
CMD_ODOM    = 0x02
CMD_GRIPPER = 0x03
CMD_HOPPER  = 0x04
CMD_ARM     = 0x05

ODOM_PAYLOAD_LEN = 16
ARM_DEFAULT_TIME_MS = 800


def _chk(cmd: int, payload: bytes) -> int:
    c = cmd ^ len(payload)
    for b in payload:
        c ^= b
    return c & 0xFF


def build_packet(cmd: int, payload: bytes) -> bytes:
    return HEADER + bytes([cmd, len(payload)]) + payload + bytes([_chk(cmd, payload)])


def _clamp(v, lo, hi):
    return max(lo, min(hi, v))


# ── node ───────────────────────────────────────────────────────────────────
class STM32BridgeNode(Node):
    def __init__(self):
        super().__init__('stm32_bridge')

        self.declare_parameter('serial_port', '/dev/ttyTHS1')
        self.declare_parameter('baud_rate', 115200)

        port = self.get_parameter('serial_port').get_parameter_value().string_value
        baud = self.get_parameter('baud_rate').get_parameter_value().integer_value

        self.get_logger().info(f'opening {port} at {baud} baud')
        self.ser = serial.Serial(port, baud, timeout=0.05)
        self.ser_lock = threading.Lock()

        self.create_subscription(Float32MultiArray, '/wheel_speeds', self._drive_cb, 10)
        self.create_subscription(Int32, '/gripper', self._gripper_cb, 10)
        self.create_subscription(Int32, '/hopper', self._hopper_cb, 10)
        self.create_subscription(Float32MultiArray, '/arm', self._arm_cb, 10)

        self.enc_pub = self.create_publisher(Int32MultiArray, '/wheel_encoders', 10)

        self._read_thread = threading.Thread(
            target=self._read_loop, daemon=True, name='serial_rx')
        self._read_thread.start()

        self.get_logger().info('stm32_bridge ready')

    def _send(self, pkt: bytes):
        with self.ser_lock:
            self.ser.write(pkt)

    # ── tx ───────────────────────────────────────────────────────────────
    def _drive_cb(self, msg: Float32MultiArray):
        if len(msg.data) < 4:
            self.get_logger().warn('wheel_speeds needs 4 elements')
            return
        speeds = [_clamp(int(v * 1000), -1000, 1000) for v in msg.data[:4]]
        self._send(build_packet(CMD_DRIVE, struct.pack('<4h', *speeds)))

    def _gripper_cb(self, msg: Int32):
        pos = _clamp(int(msg.data), 0, 100)
        self._send(build_packet(CMD_GRIPPER, bytes([pos])))

    def _hopper_cb(self, msg: Int32):
        act = _clamp(int(msg.data), 0, 2)
        self._send(build_packet(CMD_HOPPER, bytes([act])))

    def _arm_cb(self, msg: Float32MultiArray):
        if len(msg.data) < 3:
            self.get_logger().warn('arm needs at least 3 angles')
            return
        angles = [_clamp(int(a * 10), -32768, 32767) for a in msg.data[:3]]  # tenths-deg
        t = int(msg.data[3]) if len(msg.data) >= 4 else ARM_DEFAULT_TIME_MS
        self._send(build_packet(CMD_ARM, struct.pack('<3hH', *angles, t & 0xFFFF)))

    # ── rx ───────────────────────────────────────────────────────────────
    def _read_loop(self):
        buf = bytearray()
        while rclpy.ok():
            try:
                chunk = self.ser.read(64)
            except Exception as e:
                self.get_logger().error(f'serial read error: {e}')
                break
            if chunk:
                buf.extend(chunk)
                buf = self._drain(buf)

    def _drain(self, buf: bytearray) -> bytearray:
        # need at least header+cmd+len before we know the frame size
        while len(buf) >= 4:
            if buf[0] != 0xAA or buf[1] != 0x55:
                del buf[0]               # resync byte by byte
                continue
            cmd = buf[2]
            length = buf[3]
            frame_len = 4 + length + 1
            if len(buf) < frame_len:
                break                    # wait for the rest
            payload = bytes(buf[4:4 + length])
            chk = buf[frame_len - 1]
            if _chk(cmd, payload) == chk and cmd == CMD_ODOM and length == ODOM_PAYLOAD_LEN:
                ticks = struct.unpack('<4i', payload)
                out = Int32MultiArray()
                out.data = list(ticks)
                self.enc_pub.publish(out)
            del buf[:frame_len]
        return buf

    def destroy_node(self):
        self.ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = STM32BridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
