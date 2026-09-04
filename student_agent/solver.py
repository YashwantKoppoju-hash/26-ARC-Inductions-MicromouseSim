from dataclasses import dataclass
import math
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

# ==========================================
# These four parameters MUST add up to exactly 30!
# ==========================================
TOP_SPEED = 8
ACCELARATION = 7
TURN_SPEED = 5
SENSOR_RANGE = 10
VELOCITY_TOLERANCE = 1e-3

@dataclass
class  pose:
    x: float
    y: float
    theta: float

@dataclass
class Motionstate:
    velocity: float
    targetVelocity: float
    angularVelocity: float
    Acceleration: bool

@dataclass
class SensorValues:
    left: float = math.inf
    right: float = math.inf
    forward: float = math.inf
    velocity: float = 0.0

def updatePose(
    pose: pose,
    motionstate: Motionstate,
    sensorvalues: SensorValues,
    dt: float,
) -> None:
    """Update the existing pose using the latest measured motion."""
    if dt <= 0.0:
        return

    # During acceleration or braking, use the average of the previous and
    # current measured velocities. Otherwise, the current measured velocity
    # is already equal to the target within tolerance.
    if motionstate.Acceleration:
        linear_velocity = 0.5 * (
            motionstate.velocity + sensorvalues.velocity
        )
    else:
        linear_velocity = motionstate.velocity

    # Angular velocity is applied instantaneously by the simulator.
    pose.theta += motionstate.angularVelocity * dt
    pose.x += linear_velocity * math.cos(pose.theta) * dt
    pose.y += linear_velocity * math.sin(pose.theta) * dt

def updateMotionState(
    motionstate: Motionstate,
    sensorvalues: SensorValues,
) -> None:
    """Update motion state from the latest sensor velocity reading."""
    motionstate.velocity = sensorvalues.velocity
    motionstate.Acceleration = not math.isclose(
        motionstate.velocity,
        motionstate.targetVelocity,
        abs_tol=VELOCITY_TOLERANCE,
    )

class StudentSolver(Node):
    def __init__(self):
        super().__init__('student_solver')

        self.pose = pose(x=1.5, y=1.5, theta=math.pi / 2)
        self.motionstate = Motionstate(
            velocity=0.0,
            targetVelocity=0.0,
            angularVelocity=0.0,
            Acceleration=False,
        )
        self.sensorvalues = SensorValues()
        self.last_pose_update = time.monotonic()
        
        # subscriber to read sensor values (L,F,R)
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/mouse/scan',
            self.scan_callback,
            10
        )

        self.velocity_sub = self.create_subscription(
            Twist,
            '/mouse/vel',
            self.velocity_callback,
            10
        )
        
        # publisher to send movement commands
        self.cmd_pub = self.create_publisher(
            Twist,
            '/mouse/cmd_vel',
            10
        )
        
        self.get_logger().info("Student Solver Node initialized successfully.")
        self.get_logger().info(f"Stats -> Speed: {TOP_SPEED}, Accel: {ACCELARATION}, Turn: {TURN_SPEED}, Range: {SENSOR_RANGE}")

    def scan_callback(self, msg):
        """
        This function runs every time a new sensor reading is received (at 20 Hz).
        msg.ranges contains the distances:
        msg.ranges[0] -> Left ray distance
        msg.ranges[1] -> Front ray distance
        msg.ranges[2] -> Right ray distance
        """
        self.sensorvalues.left = msg.ranges[0]
        self.sensorvalues.forward = msg.ranges[1]
        self.sensorvalues.right = msg.ranges[2]

        now = time.monotonic()
        dt = now - self.last_pose_update
        updatePose(
            self.pose,
            self.motionstate,
            self.sensorvalues,
            dt,
        )
        updateMotionState(self.motionstate, self.sensorvalues)
        self.last_pose_update = now

        cmd = Twist()
        self.motionstate.targetVelocity = cmd.linear.x
        self.cmd_pub.publish(cmd)

    def velocity_callback(self, msg):
        self.sensorvalues.velocity = msg.linear.x
        self.motionstate.angularVelocity = msg.angular.z

def main(args=None):
    rclpy.init(args=args)
    node = StudentSolver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

