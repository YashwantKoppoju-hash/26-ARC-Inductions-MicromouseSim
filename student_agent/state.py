"""Shared state; this module has no ROS or simulator dependencies."""

from dataclasses import dataclass
from enum import IntEnum
import math


def wrap_angle(angle: float) -> float:
    return (angle + math.pi) % (2 * math.pi) - math.pi


@dataclass
class Pose:
    x: float = 1.5
    y: float = 1.5
    theta: float = math.pi / 2


@dataclass(frozen=True, order=True)
class Cell:
    row: int
    column: int

    @property
    def center(self):
        return self.column + 0.5, self.row + 0.5


class Direction(IntEnum):
    NORTH = 1
    EAST = 2
    SOUTH = 4
    WEST = 8

    @property
    def delta(self):
        return {1: (1, 0), 2: (0, 1), 4: (-1, 0), 8: (0, -1)}[self]

    @property
    def angle(self):
        return {1: math.pi / 2, 2: 0.0, 4: -math.pi / 2, 8: math.pi}[self]

    def rotated(self, quarter_turns):
        order = (Direction.EAST, Direction.NORTH, Direction.WEST, Direction.SOUTH)
        return order[(order.index(self) + quarter_turns) % 4]

    def neighbour(self, cell):
        dr, dc = self.delta
        return Cell(cell.row + dr, cell.column + dc)


@dataclass(frozen=True)
class SensorValues:
    left: float
    front: float
    right: float

    @property
    def valid(self):
        # A positive infinity is the conventional no-return value.
        return all(not math.isnan(d) and d > 0 for d in (self.left, self.front, self.right))


@dataclass(frozen=True)
class MotionState:
    # Engine speed states: linear_velocity is NOT ground speed after a collision.
    linear_velocity: float = 0.0
    angular_velocity: float = 0.0


@dataclass(frozen=True)
class Command:
    linear: float = 0.0
    angular: float = 0.0
