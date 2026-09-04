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
        projection = math.cos(wrap_angle(self.pose.theta - self.heading.angle))
        for turns, distance in ((1, sensors.left), (0, sensors.front), (-1, sensors.right)):
            ray = self.heading.rotated(turns)
            dr, dc = ray.delta
            x, y = self.pose.x, self.pose.y
            edge_distance = (self.cell.column + 1 - x if dc > 0 else x - self.cell.column) if dc else (
                self.cell.row + 1 - y if dr > 0 else y - self.cell.row)
            self.maze.set_edge(self.cell, ray, distance * projection < edge_distance + 0.15)

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
            self.command = self.fail('Sensor timing gap; pose is no longer reliable')
            return self.command
        self.phase_time += dt
        self.estimator.update(sensors, motion, dt)
        if self.phase != 'TURN':
            self.estimator.correct_from_ranges(sensors, self.direction, self.max_range)
        self.command = self._control(sensors, motion, dt)
        return self.command

    def _control(self, sensors, motion, dt):
        if self.phase_time > 20.0:
            return self.fail('Motion timeout in ' + self.phase)

        if self.phase == 'SENSE':
            if abs(motion.linear_velocity) > 0.008:
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
            if sensors.front <= 0.23:
                return self.fail('Front obstruction before target; stopped to avoid false odometry')
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
            elif remaining < -0.18:
                return self.fail('Overshot target cell centre')
            else:
                self.maze.set_edge(self.cell, self.direction, False)
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
        }
