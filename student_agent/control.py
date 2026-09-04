"""Wall PID, bounded motion commands, and range-corrected odometry."""

import math

from student_agent.state import Pose, wrap_angle


def clamp(value, limit):
    return max(-limit, min(limit, value))


def side_rays_reliable(pose, direction):
    # Near a cell boundary, a slightly angled side ray can hit a wall end
    # in the next cell instead of the side wall. Pause centring there.
    dr, dc = direction.delta
    forward = pose.x if dc else pose.y
    return abs(forward - round(forward)) > 0.12


class PID:
    def __init__(self, kp=3.0, ki=0.12, kd=0.45, limit=0.3):
        self.kp, self.ki, self.kd, self.limit = kp, ki, kd, limit
        self.reset()

    def reset(self):
        self.integral = 0.0
        self.previous = None
        self.derivative = 0.0

    def update(self, error, dt):
        if dt <= 0:
            return 0.0
        if abs(error) < 1e-8:
            self.reset()
            return 0.0
        if self.previous is not None:
            raw_derivative = (error - self.previous) / dt
            self.derivative += dt / (0.12 + dt) * (raw_derivative - self.derivative)
        candidate = clamp(self.integral + error * dt, 0.5)
        output = self.kp * error + self.ki * candidate + self.kd * self.derivative
        # Conditional integration: do not accumulate further into saturation.
        if abs(output) <= self.limit or error * output < 0:
            self.integral = candidate
        self.previous = error
        return clamp(output, self.limit)


class WallCentering:
    def __init__(self):
        self.pid = PID()
        self.walls = None
        self.error = 0.0

    @staticmethod
    def offset(distance):
        if distance > 0.5:
            return distance - 0.5
        if distance < 0.4:
            return distance - 0.4
        return 0.0

    def reset(self):
        self.pid.reset()
        self.walls = None
        self.error = 0.0

    def update(self, sensors, heading_error, dt):
        projection = math.cos(heading_error)
        left, right = sensors.left * projection, sensors.right * projection
        walls = (math.isfinite(left) and 0 < left < 0.85,
                 math.isfinite(right) and 0 < right < 0.85)
        if walls != self.walls:
            self.pid.reset()
            self.walls = walls
        self.error = (self.offset(left) if walls[0] else 0.0) - (self.offset(right) if walls[1] else 0.0)
        return self.pid.update(self.error, dt)


class PoseEstimator:
    def __init__(self):
        self.pose = Pose()
        self.previous_speed = 0.0
        self.translation_blocked = False

    def update(self, sensors, motion, dt):
        self.pose.theta = wrap_angle(self.pose.theta + motion.angular_velocity * dt)
        speed = (self.previous_speed + motion.linear_velocity) * 0.5
        self.previous_speed = motion.linear_velocity
        # Collisions reject translation without zeroing the engine's speed.
        self.translation_blocked = speed > 0 and sensors.front <= 0.18
        if not self.translation_blocked:
            self.pose.x += speed * math.cos(self.pose.theta) * dt
            self.pose.y += speed * math.sin(self.pose.theta) * dt

    def correct_from_ranges(self, sensors, direction, max_range):
        """Anchor to integer wall planes only while nearly cardinal.

        The maze has unit cells and axis-aligned walls. Rounding a ray's
        estimated endpoint identifies its wall plane, NOT a hidden maze cell.
        Large inconsistent corrections are rejected instead of teleporting.
        """
        heading_error = wrap_angle(self.pose.theta - direction.angle)
        if abs(heading_error) > 0.12:
            return
        for turns, distance in ((0, sensors.front), (1, sensors.left), (-1, sensors.right)):
            if turns and not side_rays_reliable(self.pose, direction):
                continue
            limit = min(max_range - 0.05, 1.8 if turns == 0 else 0.85)
            if not math.isfinite(distance) or not 0 < distance < limit:
                continue
            ray = direction.rotated(turns)
            dr, dc = ray.delta
            coordinate = self.pose.x if dc else self.pose.y
            sign = dc or dr
            perpendicular = distance * math.cos(heading_error)
            wall_plane = round(coordinate + sign * perpendicular)
            corrected = wall_plane - sign * perpendicular
            if abs(corrected - coordinate) <= 0.18:
                if dc:
                    self.pose.x = corrected
                else:
                    self.pose.y = corrected


def braking_speed(distance, acceleration, latency=0.10):
    """Speed that can stop in distance including one delayed command."""
    distance = max(0.0, distance)
    return max(0.0, math.sqrt((acceleration * latency) ** 2 + 2 * acceleration * distance)
               - acceleration * latency)
