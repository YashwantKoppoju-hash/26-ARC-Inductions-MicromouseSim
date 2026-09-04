# Pose representation

## Purpose

The solver needs an internal estimate of the robot's continuous pose. The simulator maintains the true pose internally, but the solver is intentionally separated from that private state. It must therefore construct and maintain its own representation using the motion information available through the simulator interface.

This document covers only pose representation. Maze mapping, cell tracking, stack management, and route planning will be designed separately.

## Representation choice

The pose will be a dataclass containing continuous, global x, y, and theta positions.

The representation describes the robot's physical pose, not its logical maze cell. A later design may derive a cell coordinate from the continuous pose, but the two concepts should remain distinct.

## Pose object lifecycle

The solver creates one persistent pose object when it starts. The object is initialized to the simulator's known starting location and north-facing orientation.

The same object is updated throughout the run. A new pose object is not created for every sensor reading; only the values inside the existing object change. This makes the pose a single source of truth for the solver's current estimate.

## Updating the pose

The pose update will use the requested acceleration because the acceleration is achieved instantly under this design assumption.

For each elapsed time interval, the solver updates orientation using the angular motion and advances the horizontal and vertical positions according to the resulting linear motion and updated orientation. The elapsed interval is measured from timestamps so that small variations in callback timing do not accumulate as a systematic error.

This produces dead-reckoned motion: an estimate built by integrating the robot's observed movement. Sensor readings can later be used to correct or validate this estimate when the robot reaches walls, corridors, intersections, or cell reference points.

## Coordinate conventions

The pose follows the simulator's world-coordinate convention. The origin is at the lower-left of the maze, the horizontal axis points across the maze, and the vertical axis points upward. North corresponds to an angle of pi divided by two radians.

The position remains continuous even though the maze is organized into one-unit cells. This allows the controller to reason about smooth movement while the planning layer reasons about discrete cell transitions.

## Why a dataclass

The dataclass makes the meaning of each value explicit and keeps the pose conceptually cohesive. It is easier to read, extend, and validate than an ordered list whose positions have to be remembered. It also gives the design a clear boundary: anything that needs the robot's location or orientation receives the pose object rather than a collection of unrelated values.

The runtime cost is negligible because the solver needs only one pose object. The priority is clarity and correctness, not memory optimization.

## Non-goals for this design

This pose object will not yet store:

- The robot's logical cell coordinate.
- Previously visited cells.
- Detected walls or branches.
- The exploration stack.
- A planned route.

Those belong to higher-level map and navigation structures that will consume the pose estimate later.
