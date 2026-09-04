# Design

## Pose

The robot pose is represented as continuous, global x, y, and theta positions.

The pose describes physical position and orientation. Logical maze-cell position, visited cells, walls, the exploration stack, and route planning belong to higher-level navigation structures.

## Control-first plan

The first implementation step is the kinematic control layer. The search algorithm will be selected later and will communicate through a simple abstraction: move to this cell.

The control layer will translate that request into continuous turning, driving, braking, and settling behavior. It will use the pose and measured motion to determine when the target cell has been reached. This keeps search decisions independent from motor and kinematic details.

## MotionState

MotionState represents movement information needed by the control layer:

- Current measured linear velocity.
- Target linear velocity.
- Current measured angular velocity.

Acceleration is not stored in MotionState. The simulator's acceleration behavior is reflected in the measured linear velocity, while angular velocity is applied instantaneously. A target angular velocity is therefore not needed for pose integration.

