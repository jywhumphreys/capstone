#!/usr/bin/env python3
"""Minimal HTTP server that serves the dashboard web directory."""
import os
import threading
from http.server import HTTPServer, SimpleHTTPRequestHandler

import rclpy
from rclpy.node import Node


class DashboardServerNode(Node):
    def __init__(self):
        super().__init__('dashboard_server')
        self.declare_parameter('web_dir', '')
        self.declare_parameter('port', 8888)

        web_dir = self.get_parameter('web_dir').get_parameter_value().string_value
        port    = self.get_parameter('port').get_parameter_value().integer_value

        if not web_dir or not os.path.isdir(web_dir):
            self.get_logger().error(f'web_dir not found: {web_dir}')
            return

        os.chdir(web_dir)

        handler = SimpleHTTPRequestHandler
        handler.log_message = lambda *a: None  # suppress access logs

        server = HTTPServer(('0.0.0.0', port), handler)
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()

        self.get_logger().info(
            f'Dashboard at http://0.0.0.0:{port}  (serve dir: {web_dir})')


def main(args=None):
    rclpy.init(args=args)
    node = DashboardServerNode()
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
