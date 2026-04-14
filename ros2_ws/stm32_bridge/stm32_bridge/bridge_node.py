#!/usr/bin/env python3
"""
stm32_bridge — bidirectional UART bridge between ROS2 and the STM32 drivetrain.

Subscribed topics:
  /wheel_speeds  (std_msgs/Float32MultiArray)
    data[0..3] = FL, FR, RL, RR  in range –1.0 … +1.0

Published topics:
  /wheel_encoders  (std_msgs/Int32MultiArray)
    data[0..3] = FL, FR, RL, RR  cumulative encoder ticks (int32)

Parameters:
  serial_port   (string)  default: /dev/ttyTHS0
  baud_rate     (int)     default: 115200
"""

import struct
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Int32MultiArray

try:
    import serial
except ImportError:
    raise SystemExit("pyserial not found — run: pip3 install pyserial")

# ── Protocol constants (must match comms_protocol.h) ──────────────────────
HEADER       = bytes([0xAA, 0x55])
CMD_DRIVE    = 0x01
CMD_ODOM     = 0x02
ODOM_PKT_LEN = 20   # 2 header + 1 cmd + 16 payload + 1 chk


def _checksum(cmd: int, payload: bytes) -> int:
    chk = cmd
    for b in payload:
        chk ^= b
    return chk & 0xFF


def build_drive_packet(speeds: list[int]) -> bytes:
    """speeds: list of 4 int16 in –1000 … +1000, order FL FR RL RR."""
    payload = struct.pack('>4h', *speeds)          # big-endian int16
    chk = _checksum(CMD_DRIVE, payload)
    return HEADER + bytes([CMD_DRIVE]) + payload + bytes([chk])


def parse_odom_packet(pkt: bytes):
    """pkt: 20 raw bytes starting at 0xAA.
    Returns tuple of 4 int32 ticks (FL, FR, RL, RR) or None on bad checksum."""
    if len(pkt) != ODOM_PKT_LEN:
        return None
    if pkt[0] != 0xAA or pkt[1] != 0x55 or pkt[2] != CMD_ODOM:
        return None
    payload = pkt[3:19]
    chk     = pkt[19]
    if _checksum(CMD_ODOM, payload) != chk:
        return None
    return struct.unpack('<4i', payload)            # little-endian int32


# ── Node ──────────────────────────────────────────────────────────────────
class STM32BridgeNode(Node):
    def __init__(self):
        super().__init__('stm32_bridge')

        self.declare_parameter('serial_port', '/dev/ttyTHS1')
        self.declare_parameter('baud_rate', 115200)

        port = self.get_parameter('serial_port').get_parameter_value().string_value
        baud = self.get_parameter('baud_rate').get_parameter_value().integer_value

        self.get_logger().info(f'Opening {port} at {baud} baud')
        self.ser = serial.Serial(port, baud, timeout=0.05)
        self.ser_lock = threading.Lock()

        self.speeds_sub = self.create_subscription(
            Float32MultiArray, '/wheel_speeds', self._speeds_cb, 10)
        self.enc_pub = self.create_publisher(
            Int32MultiArray, '/wheel_encoders', 10)

        self._read_thread = threading.Thread(
            target=self._read_loop, daemon=True, name='serial_rx')
        self._read_thread.start()

        self.get_logger().info('stm32_bridge ready')

    # ── TX ─────────────────────────────────────────────────────────────────
    def _speeds_cb(self, msg: Float32MultiArray):
        if len(msg.data) < 4:
            self.get_logger().warn('wheel_speeds needs 4 elements')
            return
        speeds = [max(-1000, min(1000, int(v * 1000))) for v in msg.data[:4]]
        pkt = build_drive_packet(speeds)
        with self.ser_lock:
            self.ser.write(pkt)

    # ── RX ─────────────────────────────────────────────────────────────────
    def _read_loop(self):
        buf = bytearray()
        while rclpy.ok():
            try:
                chunk = self.ser.read(64)
            except Exception as e:
                self.get_logger().error(f'Serial read error: {e}')
                break
            if chunk:
                buf.extend(chunk)
                buf = self._drain(buf)

    def _drain(self, buf: bytearray) -> bytearray:
        while len(buf) >= ODOM_PKT_LEN:
            # Find header
            idx = -1
            for i in range(len(buf) - 1):
                if buf[i] == 0xAA and buf[i + 1] == 0x55:
                    idx = i
                    break
            if idx == -1:
                return buf[-1:]      # keep last byte — may be first header byte
            if idx > 0:
                buf = buf[idx:]      # discard junk before header
            if len(buf) < ODOM_PKT_LEN:
                break
            result = parse_odom_packet(bytes(buf[:ODOM_PKT_LEN]))
            buf = buf[ODOM_PKT_LEN:]
            if result is not None:
                out = Int32MultiArray()
                out.data = list(result)
                self.enc_pub.publish(out)
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
