"""ROS adapter for online A* navigation and PID wall centring.

Run with python3 student_agent/solver.py or python3 -m student_agent.solver.
"""

import json
from pathlib import Path
import sys
import time

ROOT = str(Path(__file__).resolve().parents[1])
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

from student_agent.navigator import Navigator
from student_agent.state import Command, MotionState, SensorValues

# These four parameters MUST add up to exactly 30.
TOP_SPEED = 8
ACCELARATION = 12
TURN_SPEED = 5
SENSOR_RANGE = 5  # 5 points * 0.4 = 2 cells


class StudentSolver(Node):
    def __init__(self):
        super().__init__('student_solver')
        self.navigator = Navigator(TOP_SPEED * 0.2, ACCELARATION * 0.1,
                                   TURN_SPEED * 0.15, SENSOR_RANGE * 0.4)
        self.scan = None
        self.motion = MotionState()
        self.new_scan = self.new_motion = False
        self.previous_stamp = None
        self.last_pair = time.monotonic()
        self.last_log = 0.0
        self.last_phase = None
        self.stale_scan_seen = False
        self.scan_sub = self.create_subscription(LaserScan, '/mouse/scan', self.scan_callback, 10)
        self.velocity_sub = self.create_subscription(Twist, '/mouse/vel', self.velocity_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/mouse/cmd_vel', 10)
        self.watchdog = self.create_timer(0.1, self.check_freshness)
        self.get_logger().info('A* solver ready: pose estimation, wall PID, 30/30 stat points')

    def publish(self, command):
        msg = Twist()
        msg.linear.x, msg.angular.z = float(command.linear), float(command.angular)
        self.cmd_pub.publish(msg)

    def scan_callback(self, msg):
        if len(msg.ranges) != 3:
            self.navigator.fail('Expected exactly three range readings')
            self.publish(Command())
            return
        self.scan = msg
        self.new_scan = True
        self.consume_pair()

    def velocity_callback(self, msg):
        self.motion = MotionState(msg.linear.x, msg.angular.z)
        self.new_motion = True
        self.consume_pair()

    def consume_pair(self):
        if not (self.new_scan and self.new_motion):
            return
        self.new_scan = self.new_motion = False
        stamp = self.scan.header.stamp.sec + self.scan.header.stamp.nanosec * 1e-9
        age = self.get_clock().now().nanoseconds * 1e-9 - stamp
        if age > 0.25:
            # Draining queued scans must not count as fresh recovery evidence.
            self.stale_scan_seen = True
            self.publish(Command())
            return
        if self.previous_stamp is not None and stamp <= self.previous_stamp:
            return
        dt = 0.05 if self.previous_stamp is None else stamp - self.previous_stamp
        if self.stale_scan_seen:
            dt = max(dt, time.monotonic() - self.last_pair, 0.251)
            self.stale_scan_seen = False
        self.previous_stamp = stamp
        self.last_pair = time.monotonic()
        sensors = SensorValues(*self.scan.ranges)
        self.publish(self.navigator.step(sensors, self.motion, dt))
        if self.last_pair - self.last_log >= 1.0 or self.navigator.phase != self.last_phase:
            record = self.navigator.status()
            record.update(scan=list(self.scan.ranges),
                          scan_interval=dt, scan_age=age,
                          reported_velocity=[self.motion.linear_velocity, self.motion.angular_velocity])
            self.get_logger().info(json.dumps(record))
            self.last_log, self.last_phase = self.last_pair, self.navigator.phase

    def check_freshness(self):
        if time.monotonic() - self.last_pair > 0.25:
            self.publish(Command())


def main(args=None):
    rclpy.init(args=args)
    node = StudentSolver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish(Command())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
