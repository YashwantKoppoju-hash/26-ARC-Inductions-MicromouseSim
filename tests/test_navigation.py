"""Behavioral regressions; engine-dependent cases run in the ROS container."""

import math
from pathlib import Path
import sys
import unittest
from collections import deque
import random

from student_agent.control import PID, PoseEstimator, WallCentering, side_rays_reliable
from student_agent.navigation import AStarPlanner, MazeMap
from student_agent.navigator import Navigator
from student_agent.state import Cell, Direction, MotionState, Pose, SensorValues


class ControlTests(unittest.TestCase):
    def test_crossed_edge_cannot_be_closed_by_sensor(self):
        maze = MazeMap(3)
        maze.mark_traversed(Cell(1, 1), Direction.NORTH)
        maze.set_edge(Cell(2, 1), Direction.SOUTH, True)
        self.assertIs(maze.edge(Cell(1, 1), Direction.NORTH), False)
        self.assertIs(maze.edge(Cell(2, 1), Direction.SOUTH), False)
        self.assertEqual(maze.conflicts, 1)

    def test_oblique_and_ambiguous_ranges_do_not_write_walls(self):
        navigator = Navigator()
        navigator.pose.theta += 0.3
        navigator.observe(SensorValues(0.5, 0.5, 0.5))
        self.assertIsNone(navigator.maze.edge(navigator.cell, Direction.NORTH))
        navigator.pose.theta = math.pi / 2
        navigator.observe(SensorValues(0.65, 0.65, 0.65))
        self.assertIsNone(navigator.maze.edge(navigator.cell, Direction.NORTH))

    def test_astar_matches_breadth_first_search(self):
        rng = random.Random(42)
        for _ in range(100):
            maze = MazeMap(5)
            for r in range(5):
                for c in range(5):
                    for direction in (Direction.NORTH, Direction.EAST):
                        maze.set_edge(Cell(r, c), direction, rng.random() < 0.3)
            start, goal = Cell(0, 0), Cell(4, 4)
            queue, distances = deque([start]), {start: 0}
            while queue:
                cell = queue.popleft()
                for direction in Direction:
                    nxt = direction.neighbour(cell)
                    if maze.contains(nxt) and maze.edge(cell, direction) is not True and nxt not in distances:
                        distances[nxt] = distances[cell] + 1
                        queue.append(nxt)
            route = AStarPlanner({goal}).path(maze, start)
            self.assertEqual(len(route) - 1 if route else None, distances.get(goal))

    def test_wall_error_signs_and_band(self):
        for readings, sign in [
            ((0.6, 4.0, math.inf), 1), ((0.3, 4.0, math.inf), -1),
            ((math.inf, 4.0, 0.6), -1), ((math.inf, 4.0, 0.3), 1),
        ]:
            with self.subTest(readings=readings):
                output = WallCentering().update(SensorValues(*readings), 0, 0.05)
                self.assertGreater(output * sign, 0)
        for distance in (0.4, 0.45, 0.5):
            self.assertEqual(WallCentering().update(SensorValues(distance, 4.0, math.inf), 0, 0.05), 0)

    def test_opening_clears_integral_and_wall_end_is_rejected(self):
        controller = WallCentering()
        for _ in range(100):
            controller.update(SensorValues(0.8, 4, math.inf), 0, 0.05)
        self.assertEqual(controller.update(SensorValues(4, 4, 4), 0, 0.05), 0)
        self.assertEqual(controller.pid.integral, 0)
        self.assertFalse(side_rays_reliable(Pose(1.5, 1.99), Direction.NORTH))
        self.assertTrue(side_rays_reliable(Pose(1.5, 1.5), Direction.NORTH))

    def test_integral_does_not_wind_up_at_saturation(self):
        pid = PID()
        for _ in range(1000):
            self.assertLessEqual(abs(pid.update(10, 0.05)), pid.limit)
        self.assertEqual(pid.integral, 0)

    def test_invalid_sensor_fails_with_zero_command(self):
        navigator = Navigator()
        command = navigator.step(SensorValues(math.nan, 2, 0.5), MotionState(), 0.05)
        self.assertEqual(navigator.phase, 'FAILED')
        self.assertEqual((command.linear, command.angular), (0, 0))

    def test_reciprocal_map_and_astar_detour(self):
        maze = MazeMap(3)
        for row in range(3):
            for col in range(3):
                for direction in Direction:
                    if maze.contains(direction.neighbour(Cell(row, col))):
                        maze.set_edge(Cell(row, col), direction, False)
        maze.set_edge(Cell(0, 0), Direction.EAST, True)
        self.assertIs(maze.edge(Cell(0, 1), Direction.WEST), True)
        route = AStarPlanner({Cell(0, 2)}).path(maze, Cell(0, 0))
        self.assertEqual(len(route) - 1, 4)
        self.assertEqual(route[1], Cell(1, 0))
        # A second sensed wall changes the optimal route without a reset.
        maze.set_edge(Cell(1, 1), Direction.EAST, True)
        maze.set_edge(Cell(1, 1), Direction.SOUTH, True)
        route = AStarPlanner({Cell(0, 2)}).path(maze, Cell(0, 0))
        self.assertEqual(len(route) - 1, 6)


class EngineControlTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'simulator'))
        import sim_engine
        cls.engine = sim_engine

    def setUp(self):
        import numpy as np
        self.original_grid = self.engine.MAZE_GRID
        # A straight unit-width corridor, open for its full northward length.
        grid = np.ones((33, 33), dtype=np.int8)
        grid[1:32, 3] = 0
        self.engine.MAZE_GRID = grid
        self.config = dict(accel_rate=0.7, max_turn_rate=0.75, max_sensor_range=4.0, max_speed=1.6)

    def tearDown(self):
        self.engine.MAZE_GRID = self.original_grid

    def test_pid_physically_recenters_from_both_sides(self):
        for initial_x in (1.3, 1.7):
            with self.subTest(initial_x=initial_x):
                mouse = self.engine.VirtualMouse(self.config)
                mouse.x = initial_x
                controller = WallCentering()
                minimum_gap = 1.0
                for _ in range(200):
                    sensors = SensorValues(*mouse.calculate_ui_raycasts())
                    error = math.pi / 2 - mouse.heading
                    angular = 3 * error + controller.update(sensors, error, 0.05)
                    mouse.set_targets(0.4, max(-0.75, min(0.75, angular)))
                    for _ in range(3):
                        mouse.update(1 / 60)
                    minimum_gap = min(minimum_gap, mouse.x - 1, 2 - mouse.x)
                self.assertLess(abs(mouse.x - 1.5), 0.035)
                self.assertGreater(minimum_gap, 0.25)

    def test_collision_report_cannot_integrate_through_wall(self):
        self.engine.MAZE_GRID[4, 3] = 1
        mouse = self.engine.VirtualMouse(self.config)
        mouse.y = 1.84
        mouse.v_linear = 0.5
        mouse.set_targets(0.5, 0)
        mouse.update(0.05)
        self.assertEqual(mouse.y, 1.84)
        self.assertGreater(mouse.v_linear, 0)
        estimator = PoseEstimator()
        estimator.pose.y = mouse.y
        estimator.update(SensorValues(*mouse.calculate_ui_raycasts()), MotionState(mouse.v_linear, 0), 0.05)
        self.assertTrue(estimator.translation_blocked)
        self.assertEqual(estimator.pose.y, 1.84)


if __name__ == '__main__':
    unittest.main()
