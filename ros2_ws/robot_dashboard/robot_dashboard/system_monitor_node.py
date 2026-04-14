#!/usr/bin/env python3
"""Publishes Jetson system stats to /system_stats as JSON at 1 Hz."""
import json
import glob
import threading
import time
import psutil
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


def _read_sysfs(path, default=0.0):
    try:
        with open(path) as f:
            return float(f.read().strip())
    except Exception:
        return default


def _max_temp():
    best = 0.0
    for path in glob.glob("/sys/class/thermal/thermal_zone*/temp"):
        t = _read_sysfs(path) / 1000.0
        if t > best:
            best = t
    return best


class SystemMonitorNode(Node):
    def __init__(self):
        super().__init__("system_monitor")

        self._gpu_ema   = 0.0   # exponential moving average of GPU load
        self._gpu_lock  = threading.Lock()

        # Poll GPU at 10 Hz in background thread so we get a real average
        self._stop = False
        self._gpu_thread = threading.Thread(
            target=self._gpu_poll_loop, daemon=True, name="gpu_poll")
        self._gpu_thread.start()

        self.pub = self.create_publisher(String, "/system_stats", 10)
        self.create_timer(1.0, self._tick)
        self.get_logger().info("system_monitor ready")

    def _gpu_poll_loop(self):
        while not self._stop:
            val = _read_sysfs(
                "/sys/devices/platform/bus@0/17000000.gpu/load") / 10.0
            with self._gpu_lock:
                self._gpu_ema = 0.85 * self._gpu_ema + 0.15 * val
            time.sleep(0.1)  # 10 Hz

    def _tick(self):
        with self._gpu_lock:
            gpu = round(self._gpu_ema, 1)

        msg = String()
        msg.data = json.dumps({
            "cpu":  round(psutil.cpu_percent(interval=None), 1),
            "ram":  round(psutil.virtual_memory().percent,   1),
            "gpu":  gpu,
            "temp": round(_max_temp(),                       1),
        })
        self.pub.publish(msg)

    def destroy_node(self):
        self._stop = True
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SystemMonitorNode()
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


if __name__ == "__main__":
    main()
