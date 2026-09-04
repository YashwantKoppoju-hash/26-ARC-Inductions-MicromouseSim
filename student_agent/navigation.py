"""Reciprocal wall observations and online A*; no access to the hidden maze."""

import heapq
from itertools import count

from student_agent.state import Cell, Direction


GOALS = frozenset(Cell(r, c) for r in (7, 8) for c in (7, 8))


class MazeMap:
    def __init__(self, size=16):
        self.size = size
        self.known_mask = [[0] * size for _ in range(size)]
        self.wall_mask = [[0] * size for _ in range(size)]
        for n in range(size):
            self.set_edge(Cell(0, n), Direction.SOUTH, True)
            self.set_edge(Cell(size - 1, n), Direction.NORTH, True)
            self.set_edge(Cell(n, 0), Direction.WEST, True)
            self.set_edge(Cell(n, size - 1), Direction.EAST, True)

    def contains(self, cell):
        return 0 <= cell.row < self.size and 0 <= cell.column < self.size

    def edge(self, cell, direction):
        """None is unobserved; True is a wall; False is confirmed open."""
        if not self.contains(cell):
            return True
        if not self.known_mask[cell.row][cell.column] & direction:
            return None
        return bool(self.wall_mask[cell.row][cell.column] & direction)

    def set_edge(self, cell, direction, wall):
        adjacent = direction.neighbour(cell)
        if not self.contains(adjacent):
            wall = True
        for target, side in ((cell, direction), (adjacent, direction.rotated(2))):
            if self.contains(target):
                self.known_mask[target.row][target.column] |= side
                if wall:
                    self.wall_mask[target.row][target.column] |= side
                else:
                    self.wall_mask[target.row][target.column] &= ~side


class AStarPlanner:
    """Plan optimistically through unknown edges and replan after observations.

    Only confirmed-open edges are executed by Navigator. With positive unit
    costs the Manhattan distance to the nearest goal is admissible.
    """

    def __init__(self, goals=GOALS):
        self.goals = frozenset(goals)
        self.searches = 0

    def path(self, maze, start):
        self.searches += 1
        serial = count()
        heuristic = lambda c: min(abs(c.row - g.row) + abs(c.column - g.column) for g in self.goals)
        queue = [(heuristic(start), 0, next(serial), start)]
        cost = {start: 0}
        parent = {}
        while queue:
            _, g, _, cell = heapq.heappop(queue)
            if g != cost[cell]:
                continue
            if cell in self.goals:
                route = [cell]
                while route[-1] != start:
                    route.append(parent[route[-1]])
                return list(reversed(route))
            for direction in Direction:
                adjacent = direction.neighbour(cell)
                if not maze.contains(adjacent) or maze.edge(cell, direction) is True:
                    continue
                new_cost = g + 1
                if new_cost < cost.get(adjacent, float('inf')):
                    cost[adjacent] = new_cost
                    parent[adjacent] = cell
                    heapq.heappush(queue, (new_cost + heuristic(adjacent), new_cost, next(serial), adjacent))
        return []


def direction_between(source, target):
    for direction in Direction:
        if direction.neighbour(source) == target:
            return direction
    raise ValueError('Motion requests must name an adjacent cell')
