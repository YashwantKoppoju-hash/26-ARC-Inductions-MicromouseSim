"""Sensor-driven execution of one adjacent-cell A* move at a time."""

import math

from student_agent.control import PoseEstimator, WallCentering, braking_speed, clamp, side_rays_reliable
from student_agent.navigation import AStarPlanner, GOALS, MazeMap, direction_between
from student_agent.state import Cell, Command, Direction, wrap_angle


class Navigator:
    def __init__(self, max_speed=1.6, acceleration=0.7, max_turn=0.75, max_range=4.0):
        self.max_speed = min(max_speed, 0.55)
        self.acceleration, self.max_turn, self.max_range = acceleration, max_turn, max_range
        self.estimator = PoseEstimator()
        self.centering = WallCentering()
        self.maze = MazeMap()
        self.planner = AStarPlanner()
        self.cell = Cell(1, 1)
        self.heading = Direction.NORTH
        self.target = None
        self.direction = self.heading
        self.phase = 'SENSE'
        self.phase_time = 0.0
        self.settle_ticks = 0
        self.moves = 0
        self.reason = ''
        self.command = Command()
        self.recoveries = 0
        self.recovery_cell = None
        self.recovery_time = 0.0
        self.timing_gaps = 0
        self.timing_distance = 0.0
        self.timing_angle = 0.0
        self.last_angular = 0.0

    def timing_pause(self, gap):
        if self.phase in ('FAILED', 'COMPLETE'):
            return
        # Bounds, not an extrapolated displacement: motion during missing
        # samples is unknown. Include travel while the stop command brakes.
        if self.phase != 'TIMING_PAUSE':
            self.timing_distance = 0.0
            self.timing_angle = 0.0
            self.timing_speed = max(abs(self.estimator.previous_speed), abs(self.command.linear))
            self.timing_turn = max(abs(self.last_angular), abs(self.command.angular))
            self.timing_gaps += 1
        self.timing_distance += self.timing_speed * gap + self.timing_speed ** 2 / (2 * self.acceleration)
        self.timing_angle += self.timing_turn * gap
        self.reason = 'Waiting for fresh stationary readings after timing gap'
        self.transition('TIMING_PAUSE')
        self.command = Command()

    def resume_after_gap(self, sensors, motion, dt):
        # No odometry or map writes while uncertain. Require five successive
        # fresh stationary pairs before attempting local wall-plane matching.
        if abs(motion.linear_velocity) >= 0.008 or abs(motion.angular_velocity) >= 0.01:
            self.timing_distance += abs(motion.linear_velocity) * dt
            self.timing_angle += abs(motion.angular_velocity) * dt
            self.settle_ticks = 0
            return Command()
        self.settle_ticks += 1
        if self.settle_ticks < 5:
            return Command()
        if self.timing_angle > 0.08 or self.timing_distance > 0.35:
            self.reason = 'Paused: displacement or heading uncertainty requires relocalization/reset'
            return Command()
        direction = min(Direction, key=lambda d: abs(wrap_angle(d.angle - self.pose.theta)))
        error = wrap_angle(self.pose.theta - direction.angle)
        if abs(error) + self.timing_angle > 0.12:
            self.reason = 'Paused: heading is not reliable enough to match wall planes'
            return Command()
        candidates = {'x': [], 'y': []}
        for turns, distance in ((0, sensors.front), (1, sensors.left), (-1, sensors.right)):
            if not math.isfinite(distance) or not 0 < distance < self.max_range - 0.05:
                continue
            dr, dc = direction.rotated(turns).delta
            axis, sign = ('x', dc) if dc else ('y', dr)
            coordinate = getattr(self.pose, axis)
            projected = distance * math.cos(error)
            corrected = round(coordinate + sign * projected) - sign * projected
            if abs(corrected - coordinate) <= self.timing_distance + 0.04:
                candidates[axis].append(corrected)
        corrected_pose = {}
        for axis, values in candidates.items():
            if not values or max(values) - min(values) > 0.06:
                self.reason = 'Paused: need consistent wall evidence on both position axes'
                return Command()
            corrected_pose[axis] = sum(values) / len(values)
        occupied = Cell(math.floor(corrected_pose['y']), math.floor(corrected_pose['x']))
        if occupied not in (self.cell, self.target):
            self.reason = 'Paused: wall evidence lies outside the current move'
            return Command()
        self.pose.x, self.pose.y = corrected_pose['x'], corrected_pose['y']
        self.estimator.previous_speed = 0.0
        self.direction = direction
        return self.recover('Fresh readings restored; recentering after timing gap')

    def recover(self, reason):
        """Recenter inside the estimated occupied cell before observing again."""
        occupied = Cell(math.floor(self.pose.y), math.floor(self.pose.x))
        if occupied not in (self.cell, self.target) or not self.maze.contains(occupied):
            return self.fail('Recovery pose outside current move')
        self.recovery_cell = occupied
        self.recovery_time = 0.0
        self.recoveries += 1
        self.reason = reason
        self.transition('RECOVER_BRAKE')
        return Command()

    def recovery_control(self, sensors, motion, dt):
        self.recovery_time += dt
        if self.recovery_time > 45:
            return self.fail('Recovery timeout; cannot establish clearance')
        tx, ty = self.recovery_cell.center
        dx, dy = tx - self.pose.x, ty - self.pose.y
        stopped = abs(motion.linear_velocity) < 0.008 and abs(motion.angular_velocity) < 0.01
        if self.phase == 'RECOVER_BRAKE':
            if not stopped:
                return Command()
            if math.hypot(dx, dy) < 0.06:
                if self.recovery_cell != self.cell:
                    self.maze.mark_traversed(self.cell, direction_between(self.cell, self.recovery_cell))
                    self.moves += 1
                self.cell = self.recovery_cell
                self.target = None
                self.heading = self.direction = min(Direction, key=lambda d: abs(wrap_angle(d.angle - self.pose.theta)))
                self.reason = ''
                self.transition('SENSE')
                return Command()
            # A short reverse is allowed only toward this cell's interior.
            # The maze has no internal obstacles within a cell; never reverse
            # across an unobserved rear edge.
            backward = -(dx * math.cos(self.pose.theta) + dy * math.sin(self.pose.theta))
            if sensors.front < 0.3 and backward > 0.06:
                self.reverse_origin = (self.pose.x, self.pose.y)
                self.transition('RECOVER_BACK')
            else:
                self.direction = (Direction.EAST if dx > 0 else Direction.WEST) if abs(dx) > abs(dy) else (
                    Direction.NORTH if dy > 0 else Direction.SOUTH)
                self.transition('RECOVER_TURN')
            return Command()
        if self.phase == 'RECOVER_BACK':
            distance = math.hypot(self.pose.x - self.reverse_origin[0], self.pose.y - self.reverse_origin[1])
            nx = self.pose.x - 0.12 * math.cos(self.pose.theta)
            ny = self.pose.y - 0.12 * math.sin(self.pose.theta)
            r, c = self.recovery_cell.row, self.recovery_cell.column
            if distance >= 0.12 or sensors.front >= 0.4 or not (c + 0.2 < nx < c + 0.8 and r + 0.2 < ny < r + 0.8):
                self.transition('RECOVER_BRAKE')
                return Command()
            return Command(linear=-0.08)
        if self.phase == 'RECOVER_TURN':
            error = wrap_angle(self.direction.angle - self.pose.theta)
            if abs(error) > 0.02:
                return Command(angular=clamp(2 * error, 0.35))
            if stopped:
                self.transition('RECOVER_MOVE')
            return Command()
        dr, dc = self.direction.delta
        remaining = dx * dc + dy * dr
        if remaining < 0.035 or sensors.front < 0.3:
            self.transition('RECOVER_BRAKE')
            return Command()
        return Command(min(0.10, 0.6 * remaining),
                       clamp(2 * wrap_angle(self.direction.angle - self.pose.theta), 0.25))

    @property
    def pose(self):
        return self.estimator.pose

    def transition(self, phase):
        self.phase, self.phase_time = phase, 0.0
        self.settle_ticks = 0
        self.centering.reset()

    def fail(self, reason):
        self.reason = reason
        self.transition('FAILED')
        return Command()

    def observe(self, sensors):
        error = wrap_angle(self.pose.theta - self.heading.angle)
        if abs(error) > 0.12:
            return
        projection = math.cos(error)
        for turns, distance in ((1, sensors.left), (0, sensors.front), (-1, sensors.right)):
            ray = self.heading.rotated(turns)
            dr, dc = ray.delta
            x, y = self.pose.x, self.pose.y
            edge_distance = (self.cell.column + 1 - x if dc > 0 else x - self.cell.column) if dc else (
                self.cell.row + 1 - y if dr > 0 else y - self.cell.row)
            # Ray must cross the intended edge away from a corner. Otherwise
            # an adjacent wall end can masquerade as this cell's wall.
            lateral = y - self.cell.row if dc else x - self.cell.column
            drift = edge_distance * math.tan(error) * (dc or -dr)
            crossing = lateral + drift
            if edge_distance <= 0 or not 0.15 < crossing < 0.85:
                continue
            projected = distance * projection
            if abs(projected - edge_distance) <= 0.10:
                self.maze.set_edge(self.cell, ray, True)
            elif projected > edge_distance + 0.20:
                self.maze.set_edge(self.cell, ray, False)

    def remaining(self):
        tx, ty = self.target.center
        dr, dc = self.direction.delta
        return (tx - self.pose.x) * dc + (ty - self.pose.y) * dr

    def step(self, sensors, motion, dt):
        if self.phase in ('FAILED', 'COMPLETE'):
            self.command = Command()
            return self.command
        if not sensors.valid or not all(math.isfinite(v) for v in (motion.linear_velocity, motion.angular_velocity, dt)):
            self.command = self.fail('Invalid sensor or velocity sample')
            return self.command
        if not 0 < dt <= 0.25:
            self.timing_pause(max(dt, 0.25))
            return self.command
        if self.phase == 'TIMING_PAUSE':
            self.command = self.resume_after_gap(sensors, motion, dt)
            return self.command
        self.last_angular = motion.angular_velocity
        self.phase_time += dt
        self.estimator.update(sensors, motion, dt)
        if self.phase not in ('TURN', 'RECOVER_TURN', 'RECOVER_BACK', 'RECOVER_BRAKE'):
            self.estimator.correct_from_ranges(sensors, self.direction, self.max_range)
        self.command = self._control(sensors, motion, dt)
        return self.command

    def _control(self, sensors, motion, dt):
        if self.phase.startswith('RECOVER_'):
            return self.recovery_control(sensors, motion, dt)
        if self.phase_time > 20.0:
            return self.fail('Motion timeout in ' + self.phase)

        if self.phase == 'SENSE':
            if abs(motion.linear_velocity) > 0.008:
                return Command()
            error = wrap_angle(self.heading.angle - self.pose.theta)
            if abs(error) > 0.008:
                return Command(angular=clamp(3.0 * error, self.max_turn))
            if abs(motion.angular_velocity) >= 0.01:
                return Command()
            self.observe(sensors)
            if self.cell in GOALS:
                self.transition('COMPLETE')
                return Command()
            path = self.planner.path(self.maze, self.cell)
            if len(path) < 2:
                return self.fail('No route to the goal in the observed map')
            self.target = path[1]
            self.direction = direction_between(self.cell, self.target)
            self.transition('TURN')
            return Command()

        if self.phase == 'TURN':
            if abs(motion.linear_velocity) > 0.008:
                return Command()
            error = wrap_angle(self.direction.angle - self.pose.theta)
            if abs(error) > 0.008:
                self.settle_ticks = 0
                return Command(angular=clamp(3.0 * error, self.max_turn))
            self.settle_ticks += 1
            if self.settle_ticks >= 3 and abs(motion.angular_velocity) < 0.01:
                self.heading = self.direction
                self.estimator.correct_from_ranges(sensors, self.heading, self.max_range)
                self.observe(sensors)
                # A* may propose an unobserved edge. Confirm it after turning.
                if self.maze.edge(self.cell, self.direction) is not False:
                    self.target = None
                    self.transition('SENSE')
                else:
                    self.transition('DRIVE')
            return Command()

        if self.phase == 'DRIVE':
            if self.target in GOALS and 7 <= self.pose.x <= 9 and 7 <= self.pose.y <= 9:
                self.transition('COMPLETE')
                return Command()
            remaining = self.remaining()
            if remaining <= 0.025:
                self.transition('STOP')
                return Command()
            if sensors.front < 0.3:
                return self.recover('Front clearance below 0.3')
            speed = min(self.max_speed,
                        braking_speed(remaining - 0.02, self.acceleration),
                        braking_speed(sensors.front - 0.25, self.acceleration))
            error = wrap_angle(self.direction.angle - self.pose.theta)
            if side_rays_reliable(self.pose, self.direction):
                steering = self.centering.update(sensors, error, dt)
            else:
                self.centering.reset()
                steering = 0.0
            if not self.centering.walls or not any(self.centering.walls):
                tx, ty = self.target.center
                dr, dc = self.direction.delta
                lateral = -(tx - self.pose.x) * dr + (ty - self.pose.y) * dc
                error += clamp(math.atan2(lateral, max(remaining, 0.3)), 0.15)
            return Command(speed, clamp(3.0 * error + steering, self.max_turn))

        if self.phase == 'STOP':
            if abs(motion.linear_velocity) > 0.008:
                return Command()
            remaining = self.remaining()
            if remaining > 0.08:
                self.transition('DRIVE')
            elif remaining < -0.06:
                return self.recover('Overshot target cell centre')
            else:
                self.maze.mark_traversed(self.cell, self.direction)
                self.cell = self.target
                self.target = None
                self.moves += 1
                self.transition('SENSE')
            return Command()
        return self.fail('Unknown control state')

    def status(self):
        return {
            'phase': self.phase, 'reason': self.reason,
            'cell': [self.cell.row, self.cell.column],
            'target': None if self.target is None else [self.target.row, self.target.column],
            'pose': [self.pose.x, self.pose.y, self.pose.theta],
            'command': [self.command.linear, self.command.angular],
            'centering_error': self.centering.error,
            'translation_blocked': self.estimator.translation_blocked,
            'moves': self.moves, 'astar_searches': self.planner.searches,
            'map_conflicts': self.maze.conflicts,
            'recoveries': self.recoveries,
            'timing_gaps': self.timing_gaps,
            'timing_uncertainty': [self.timing_distance, self.timing_angle],
        }
