#!/usr/bin/env python3
"""
drive_node — mecanum inverse kinematics.

Subscribes:  /cmd_vel            (geometry_msgs/Twist)
Publishes:   /wheel_speeds       (std_msgs/Float32MultiArray)
               data[0..3] = FL, FR, RL, RR  in –1.0 … +1.0

Mecanum layout (top view, rollers at 45°):
    FL ↗  ↖ FR
    RL ↖  ↗ RR

Kinematic equations (standard mecanum):
    v_FL = ( vx - vy - (lx + ly) * wz ) / R
    v_FR = ( vx + vy + (lx + ly) * wz ) / R
    v_RL = ( vx + vy - (lx + ly) * wz ) / R
    v_RR = ( vx - vy + (lx + ly) * wz ) / R

Then normalise so the fastest wheel is ±1.0, preserving direction ratios.

Parameters:
    wheel_radius   (m)     default 0.05   (50 mm)
    wheelbase_x    (m)     default 0.15   half front-back distance
    wheelbase_y    (m)     default 0.15   half left-right distance
    max_lin_vel    (m/s)   default 0.5    used for velocity clamping only
    max_ang_vel    (rad/s) default 2.0

If a wheel direction appears backwards, flip the sign with the
`invert_fl`, `invert_fr`, `invert_rl`, `invert_rr` bool parameters
rather than rewiring hardware.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray


class DriveNode(Node):
    def __init__(self):
        super().__init__('drive_node')

        self.declare_parameter('wheel_radius',  0.05)
        self.declare_parameter('wheelbase_x',   0.15)
        self.declare_parameter('wheelbase_y',   0.15)
        self.declare_parameter('max_lin_vel',   0.5)
        self.declare_parameter('max_ang_vel',   2.0)
        self.declare_parameter('invert_fl', False)
        self.declare_parameter('invert_fr', False)
        self.declare_parameter('invert_rl', False)
        self.declare_parameter('invert_rr', False)

        self.R   = self.get_parameter('wheel_radius').get_parameter_value().double_value
        self.lx  = self.get_parameter('wheelbase_x').get_parameter_value().double_value
        self.ly  = self.get_parameter('wheelbase_y').get_parameter_value().double_value
        self.max_lin = self.get_parameter('max_lin_vel').get_parameter_value().double_value
        self.max_ang = self.get_parameter('max_ang_vel').get_parameter_value().double_value
        self.inv = [
            -1.0 if self.get_parameter('invert_fl').get_parameter_value().bool_value else 1.0,
            -1.0 if self.get_parameter('invert_fr').get_parameter_value().bool_value else 1.0,
            -1.0 if self.get_parameter('invert_rl').get_parameter_value().bool_value else 1.0,
            -1.0 if self.get_parameter('invert_rr').get_parameter_value().bool_value else 1.0,
        ]

        self.sub = self.create_subscription(
            Twist, '/cmd_vel', self._cmd_vel_cb, 10)
        self.pub = self.create_publisher(
            Float32MultiArray, '/wheel_speeds', 10)

        self.get_logger().info(
            f'drive_node ready  R={self.R} m  lx={self.lx} m  ly={self.ly} m')

    def _cmd_vel_cb(self, msg: Twist):
        # Clamp inputs
        vx = max(-self.max_lin, min(self.max_lin, msg.linear.x))
        vy = max(-self.max_lin, min(self.max_lin, msg.linear.y))
        wz = max(-self.max_ang, min(self.max_ang, msg.angular.z))

        L = self.lx + self.ly

        # Wheel angular velocities (rad/s)
        fl = (vx - vy - wz * L) / self.R
        fr = (vx + vy + wz * L) / self.R
        rl = (vx + vy - wz * L) / self.R
        rr = (vx - vy + wz * L) / self.R

        # Normalise against max_lin so pure forward at max speed = 1.0.
        # Combined commands that exceed 1.0 on any wheel are clipped proportionally.
        ref = self.max_lin / self.R
        if ref == 0.0:
            return

        wheels = [fl / ref, fr / ref, rl / ref, rr / ref]

        # Clip proportionally if any wheel exceeds ±1.0
        peak = max(abs(w) for w in wheels)
        if peak > 1.0:
            wheels = [w / peak for w in wheels]

        # Apply direction inversions
        wheels = [w * s for w, s in zip(wheels, self.inv)]

        out = Float32MultiArray()
        out.data = [float(w) for w in wheels]
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = DriveNode()
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
