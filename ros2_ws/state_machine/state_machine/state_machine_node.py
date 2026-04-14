#!/usr/bin/env python3
"""
state_machine — robot behaviour state machine.

States:
  IDLE     waiting for command
  PATROL   random wander, scanning for trash
  APPROACH target acquired, closing in
  COLLECT  placeholder (arm pick-up not yet implemented)
  DEPOSIT  placeholder (return to bin)
  FETCH    dead-reckon to a fixed location (beer run, etc.)
  TELEOP   manual override — passes /cmd_vel_teleop straight through

External commands (publish a string to /state_cmd):
  "patrol"  "fetch"  "idle"  "teleop"

Topics published:
  /cmd_vel      geometry_msgs/Twist
  /robot_state  std_msgs/String   — current state name, 20 Hz

Topics subscribed:
  /detections       vision_msgs/Detection2DArray  (from yolo_detector)
  /state_cmd        std_msgs/String
  /cmd_vel_teleop   geometry_msgs/Twist           (used in TELEOP state)

COCO trash classes targeted by default:
  bottle(39) wine glass(40) cup(41) fork(42) knife(43)
  spoon(44) bowl(45) book(73) scissors(76)
"""

import random
from enum import Enum

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from vision_msgs.msg import Detection2DArray


COCO_NAMES = {
    '0': 'person', '39': 'bottle', '40': 'wine glass', '41': 'cup',
    '42': 'fork', '43': 'knife', '44': 'spoon', '45': 'bowl',
    '73': 'book', '76': 'scissors',
}


class State(Enum):
    IDLE     = 'idle'
    PATROL   = 'patrol'
    APPROACH = 'approach'
    COLLECT  = 'collect'
    DEPOSIT  = 'deposit'
    FETCH    = 'fetch'
    TELEOP   = 'teleop'


DEFAULT_TRASH_CLASSES = {'39', '40', '41', '42', '43', '44', '45', '73', '76'}


class StateMachineNode(Node):
    def __init__(self):
        super().__init__('state_machine')

        self.declare_parameter('patrol_speed',         0.3)
        self.declare_parameter('patrol_turn_speed',    0.8)
        self.declare_parameter('patrol_straight_time', 2.5)
        self.declare_parameter('patrol_turn_time',     1.5)
        self.declare_parameter('approach_speed',       0.25)
        self.declare_parameter('approach_kp',          1.2)
        self.declare_parameter('approach_stop_area',   0.12)
        self.declare_parameter('image_width',          640.0)
        self.declare_parameter('image_height',         480.0)
        self.declare_parameter('collect_duration',     3.0)
        self.declare_parameter('deposit_duration',     3.0)
        self.declare_parameter('fetch_speed',          0.3)
        self.declare_parameter('fetch_duration',       5.0)
        self.declare_parameter('fetch_turn_angle',     0.0)
        self.declare_parameter('trash_classes', list(DEFAULT_TRASH_CLASSES))
        self.declare_parameter('loop_hz',       20.0)

        def p(name):
            return self.get_parameter(name).value

        self.patrol_speed         = p('patrol_speed')
        self.patrol_turn_speed    = p('patrol_turn_speed')
        self.patrol_straight_time = p('patrol_straight_time')
        self.patrol_turn_time     = p('patrol_turn_time')
        self.approach_speed       = p('approach_speed')
        self.approach_kp          = p('approach_kp')
        self.approach_stop_area   = p('approach_stop_area')
        self.img_w                = p('image_width')
        self.img_h                = p('image_height')
        self.collect_duration     = p('collect_duration')
        self.deposit_duration     = p('deposit_duration')
        self.fetch_speed          = p('fetch_speed')
        self.fetch_duration       = p('fetch_duration')
        self.fetch_turn_angle     = p('fetch_turn_angle')
        self.trash_classes        = set(p('trash_classes'))

        self.state               = State.IDLE
        self._state_start        = self.get_clock().now()
        self._patrol_phase       = 'straight'
        self._patrol_turn_dir    = 1.0
        self._latest_detections  = []
        self._teleop_twist       = Twist()
        self._teleop_recv_time   = None
        self._log_times          = {}   # key → rclpy.Time, for throttled logging

        self.cmd_pub   = self.create_publisher(Twist,  '/cmd_vel',     10)
        self.state_pub = self.create_publisher(String, '/robot_state', 10)

        self.create_subscription(Detection2DArray, '/detections',     self._det_cb,    10)
        self.create_subscription(String,           '/state_cmd',      self._cmd_cb,    10)
        self.create_subscription(Twist,            '/cmd_vel_teleop', self._teleop_cb, 10)

        self.create_timer(1.0 / p('loop_hz'), self._tick)
        self.get_logger().info('state_machine ready  state=IDLE')

    # ── Logging helpers ───────────────────────────────────────────────────────
    def _tlog(self, key, msg, period=1.0):
        """Log msg at most once every `period` seconds for the given key."""
        now  = self.get_clock().now()
        last = self._log_times.get(key)
        if last is None or (now - last).nanoseconds * 1e-9 >= period:
            self._log_times[key] = now
            self.get_logger().info(msg)

    # ── Helpers ───────────────────────────────────────────────────────────────
    def _transition(self, new_state: State):
        if new_state == self.state:
            return
        self.get_logger().info(f'state  {self.state.value} → {new_state.value}')
        self.state        = new_state
        self._state_start = self.get_clock().now()
        self._log_times.clear()
        if new_state == State.PATROL:
            self._patrol_phase = 'straight'
        if new_state == State.TELEOP:
            self._teleop_twist     = Twist()
            self._teleop_recv_time = None

    def _elapsed(self) -> float:
        return (self.get_clock().now() - self._state_start).nanoseconds * 1e-9

    def _publish(self, vx=0.0, vy=0.0, wz=0.0):
        t = Twist()
        t.linear.x  = float(vx)
        t.linear.y  = float(vy)
        t.angular.z = float(wz)
        self.cmd_pub.publish(t)

    def _stop(self):
        self._publish()

    def _best_trash(self):
        best, best_score = None, 0.0
        for det in self._latest_detections:
            for hyp in det.results:
                if (hyp.hypothesis.class_id in self.trash_classes
                        and hyp.hypothesis.score > best_score):
                    best_score = hyp.hypothesis.score
                    best = det
        return best

    def _det_label(self, det):
        hyp = det.results[0].hypothesis if det.results else None
        if hyp is None:
            return 'unknown'
        name = COCO_NAMES.get(hyp.class_id, f'cls{hyp.class_id}')
        return f'{name} {hyp.score*100:.0f}%'

    # ── Callbacks ─────────────────────────────────────────────────────────────
    def _det_cb(self, msg: Detection2DArray):
        self._latest_detections = msg.detections

    def _cmd_cb(self, msg: String):
        cmd = msg.data.strip().lower()
        mapping = {
            'patrol': State.PATROL,
            'fetch':  State.FETCH,
            'idle':   State.IDLE,
            'teleop': State.TELEOP,
        }
        if cmd in mapping:
            self.get_logger().info(f'cmd received: {cmd}')
            self._transition(mapping[cmd])
        else:
            self.get_logger().warn(f'unknown cmd: "{cmd}"')

    def _teleop_cb(self, msg: Twist):
        self._teleop_twist     = msg
        self._teleop_recv_time = self.get_clock().now()

    # ── Main tick ─────────────────────────────────────────────────────────────
    def _tick(self):
        msg = String()
        msg.data = self.state.value
        self.state_pub.publish(msg)

        dispatch = {
            State.IDLE:     self._tick_idle,
            State.PATROL:   self._tick_patrol,
            State.APPROACH: self._tick_approach,
            State.COLLECT:  self._tick_collect,
            State.DEPOSIT:  self._tick_deposit,
            State.FETCH:    self._tick_fetch,
            State.TELEOP:   self._tick_teleop,
        }
        dispatch[self.state]()

    # ── State handlers ────────────────────────────────────────────────────────
    def _tick_idle(self):
        self._stop()

    def _tick_patrol(self):
        target = self._best_trash()
        if target is not None:
            self.get_logger().info(
                f'patrol  target acquired: {self._det_label(target)} — switching to approach')
            self._transition(State.APPROACH)
            return

        elapsed = self._elapsed()

        if self._patrol_phase == 'straight':
            self._publish(vx=self.patrol_speed)
            self._tlog('patrol_status',
                f'patrol  driving straight  {elapsed:.1f}/{self.patrol_straight_time:.1f}s'
                + (f'  {len(self._latest_detections)} object(s) in view' if self._latest_detections else '  no objects in view'),
                period=1.0)
            if elapsed >= self.patrol_straight_time:
                self._patrol_phase    = 'turn'
                self._patrol_turn_dir = random.choice([-1.0, 1.0])
                self._state_start     = self.get_clock().now()
                direction = 'left' if self._patrol_turn_dir > 0 else 'right'
                self.get_logger().info(f'patrol  turning {direction} for {self.patrol_turn_time:.1f}s')
        else:
            self._publish(wz=self.patrol_turn_speed * self._patrol_turn_dir)
            direction = 'left' if self._patrol_turn_dir > 0 else 'right'
            self._tlog('patrol_status',
                f'patrol  turning {direction}  {elapsed:.1f}/{self.patrol_turn_time:.1f}s',
                period=1.0)
            if elapsed >= self.patrol_turn_time:
                self._patrol_phase = 'straight'
                self._state_start  = self.get_clock().now()
                self.get_logger().info('patrol  resuming straight')

    def _tick_approach(self):
        target = self._best_trash()
        if target is None:
            self.get_logger().info('approach  lost target — resuming patrol')
            self._transition(State.PATROL)
            return

        cx        = target.bbox.center.position.x
        bw        = target.bbox.size_x
        bh        = target.bbox.size_y
        area_frac = (bw * bh) / (self.img_w * self.img_h)
        error     = (cx - self.img_w / 2.0) / (self.img_w / 2.0)
        wz        = -self.approach_kp * error
        vx        = self.approach_speed * (1.0 - 0.5 * abs(error))

        self._tlog('approach_status',
            f'approach  {self._det_label(target)}'
            f'  area={area_frac*100:.1f}%/{self.approach_stop_area*100:.0f}%'
            f'  err={error:+.2f}'
            f'  vx={vx:.2f} wz={wz:.2f}',
            period=0.5)

        if area_frac >= self.approach_stop_area:
            self.get_logger().info(
                f'approach  close enough ({area_frac*100:.1f}% of frame) → collect')
            self._transition(State.COLLECT)
            return

        self._publish(vx=vx, wz=wz)

    def _tick_collect(self):
        self._stop()
        self._tlog('collect_status',
            f'collect  waiting  {self._elapsed():.1f}/{self.collect_duration:.1f}s  (placeholder)',
            period=1.0)
        if self._elapsed() >= self.collect_duration:
            self.get_logger().info('collect  done (placeholder) → deposit')
            self._transition(State.DEPOSIT)

    def _tick_deposit(self):
        self._publish(vx=-0.15)
        self._tlog('deposit_status',
            f'deposit  reversing  {self._elapsed():.1f}/{self.deposit_duration:.1f}s  (placeholder)',
            period=1.0)
        if self._elapsed() >= self.deposit_duration:
            self.get_logger().info('deposit  done (placeholder) → patrol')
            self._transition(State.PATROL)

    def _tick_fetch(self):
        elapsed   = self._elapsed()
        turn_time = (abs(self.fetch_turn_angle) / self.patrol_turn_speed
                     if self.fetch_turn_angle != 0.0 else 0.0)

        if elapsed < turn_time:
            sign = 1.0 if self.fetch_turn_angle > 0.0 else -1.0
            direction = 'left' if sign > 0 else 'right'
            self._publish(wz=self.patrol_turn_speed * sign)
            self._tlog('fetch_status',
                f'fetch  turning {direction}  {elapsed:.1f}/{turn_time:.1f}s',
                period=1.0)
        elif elapsed < turn_time + self.fetch_duration:
            drive_elapsed = elapsed - turn_time
            self._publish(vx=self.fetch_speed)
            self._tlog('fetch_status',
                f'fetch  driving  {drive_elapsed:.1f}/{self.fetch_duration:.1f}s'
                f'  at {self.fetch_speed:.2f} m/s',
                period=1.0)
        else:
            self._stop()
            self.get_logger().info('fetch  complete → idle')
            self._transition(State.IDLE)

    def _tick_teleop(self):
        # Watchdog: stop if no fresh cmd received within 0.5s
        if self._teleop_recv_time is not None:
            age = (self.get_clock().now() - self._teleop_recv_time).nanoseconds * 1e-9
            if age > 0.5:
                self._stop()
                self._tlog('teleop_watchdog', 'teleop  watchdog: no input, stopping', period=2.0)
                return
        else:
            self._stop()
            return
        t = self._teleop_twist
        self.cmd_pub.publish(t)
        vx, vy, wz = t.linear.x, t.linear.y, t.angular.z
        if vx != 0.0 or vy != 0.0 or wz != 0.0:
            self._tlog('teleop_input',
                f'teleop  vx={vx:.2f} vy={vy:.2f} wz={wz:.2f}',
                period=0.5)


def main(args=None):
    rclpy.init(args=args)
    node = StateMachineNode()
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
