# Design

## Pose

The robot pose is represented as continuous, global x, y, and theta positions. State will be encapsulated in objects whose structure is defined explicitly with dataclasses.

The pose describes physical position and orientation. Logical maze-cell position, visited cells, walls, the exploration stack, and route planning belong to higher-level navigation structures.

## Control-first plan

The first implementation step is the kinematic control layer. The search algorithm will be selected later and will communicate through a simple abstraction: move to this cell.

The control layer will translate that request into continuous turning, driving, braking, and settling behavior. It will use the pose and measured motion to determine when the target cell has been reached. This keeps search decisions independent from motor and kinematic details.

## MotionState

MotionState is a dataclass object that encapsulates the movement state needed by the control layer:

- The previous measured linear velocity used as the start of the current integration interval.
- Target linear velocity.
- Current measured angular velocity.
- A Boolean acceleration flag: true while measured linear velocity is still approaching target linear velocity, and false once the target is reached.

The acceleration flag should be based on a small tolerance. It identifies when the robot is still accelerating or braking rather than travelling at its target linear velocity.

## SensorValues

SensorValues is a dataclass object that encapsulates the latest observations from the robot: left, front, and right ray distances, together with the measured linear velocity available from the simulator.

The scan callback updates the three ray values first. The measured linear velocity arrives through the simulator's velocity interface and updates the sensor-values object. The measured angular velocity is retained in MotionState. The control layer can therefore consume the newest available measurements without mixing them with pose or planning state.

## Pose updates from MotionState

MotionState and SensorValues will be passed together to update the Pose object. SensorValues supplies the current measured velocity, while MotionState supplies the previous measured velocity for the start of the interval. When the acceleration flag is true, the pose update uses their average to calculate distance travelled. When the flag is false, the values agree within tolerance and the stored MotionState velocity is sufficient.

After the pose update, a separate motion-state update replaces MotionState's stored velocity with the current measured velocity so it becomes the previous value for the next interval. That separate update also computes the acceleration flag from MotionState's velocity and target velocity. The pose update uses MotionState's measured angular velocity to update theta, then uses the resulting orientation to update global x and y.

Acceleration magnitude is not stored in MotionState. The simulator's acceleration behavior is reflected in the measured linear velocity and the acceleration flag, while angular velocity is applied instantaneously. A target angular velocity is therefore not needed for pose integration.
