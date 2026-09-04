# Design Spec #2

## Objective

Navigate a fresh, unknown 16×16 maze from world position (1.5, 1.5), initially facing north, into one of the four central goal cells. A* selects the route; sensor feedback controls physical movement. Each process starts with an empty map. There is no saved-map speed run, solver reset, or reverse recovery.

## Simulator interface

The simulator publishes three robot-relative rays (left, front, right) and its internal linear/angular speed states at 20 Hz. Its physics runs at approximately 60 Hz. Commands are forward speed and angular velocity, not position or absolute angle.

Linear speed approaches the requested value subject to acceleration. Angular speed changes immediately after the command is received and is capped. A collision rejects the position update without zeroing the reported linear speed. Therefore, /mouse/vel cannot be treated as ground displacement after contact.

Stats remain speed 8, acceleration 7, turn 5, range 10: exactly 30 points. Their physical limits are 1.6 world units/s, 0.7 world units/s², 0.75 radians/s, and 4 world units of ray range. The controller currently caps forward speed at 0.55 units/s. Random maze generation is enabled.

## State and measurements

State definitions live in student_agent/state.py and have no ROS dependencies. Pose stores continuous global x, y, theta. Cell stores row and column; row increases with y and column with x. Direction is a discrete cardinal heading. SensorValues stores the latest ranges. MotionState stores reported speed values and Command stores the requested speed values.

The pose estimator integrates reported angular velocity and averages consecutive linear-speed samples. When front clearance indicates contact, it inhibits forward integration even if the reported speed is positive. This is a conservative guard, not a general collision detector.

Nearby walls provide position corrections: projecting a range onto a cardinal axis identifies the nearest integer wall plane. Corrections larger than 0.18 world units are rejected. This relies on the known unit grid and a reasonably accurate existing estimate; it is not arbitrary global localization. Theta remains an estimate, not an exact compass reading.

## Sensor-to-control layer

The centring controller uses a 0.4–0.5 acceptable side-distance band. For one wall, a distance above 0.5 produces an error toward that wall; below 0.4 produces an error away. Inside the band there is no distance error. Left and right contributions have opposite signs. With two walls in a one-unit corridor, this balances the mouse around the centre.

The PID produces angular velocity. Initial gains are proportional 3.0, integral 0.12, derivative 0.45. Its output is limited to ±0.30 radians/s. The derivative is filtered with a 0.12-second time constant; the integral is bounded and stops accumulating into saturation. History resets in the acceptable band, on changes of visible walls, and during stationary turns.

A side range above 0.85 is treated as an opening rather than a wall to approach. Side feedback and side-based pose correction pause within 0.12 units of a cell boundary because a ray can hit a wall end in the next cell. Heading feedback remains active. Where side walls are absent, the controller also steers toward the target cell centre. The combined angular command respects the simulator limit.

## Motion and collision behavior

The controller handles sensing, turning, driving, and stopping as explicit states. Mapping and planning occur at stationary cell centres, with a second observation after turning. A planned edge must be observed open before driving through it.

Turning waits for forward speed to fall below 0.008 units/s and uses heading-error feedback. Driving combines cell-centre distance, front clearance, and braking distance, allowing 0.10 seconds for command latency. The front stopping clearance is 0.25, greater than the mouse radius of 0.15.

An unexpected obstruction stops the solver with a reason rather than allowing a zero-speed drive state to wait forever. Invalid measurements, a substantial sensor timing gap, or a motion state lasting over 20 seconds also stop execution. Failures require diagnosis; the solver does not invent a position or silently reset its map.

Goal entry is detected during the final move. Tests independently require the engine's actual position to enter the goal; the solver's own completion flag alone cannot pass a test.

## Solver and planner logic

The maze keeps separate known-edge and wall bitmasks. Every observation is mirrored onto the neighbour's opposite edge. The planner runs A* with unit step costs and minimum Manhattan distance to the four goal cells. Unknown edges are optimistically traversable during planning; confirmed walls are blocked. New observations trigger replanning. A* is optimal for each current optimistic map, but the total exploratory journey need not be globally shortest.

The ROS entry point is student_agent/solver.py. It pairs fresh scan and velocity callbacks, uses the scan timestamp for elapsed time, publishes the command, and logs status periodically and at state transitions. Twist velocity messages have no timestamp, so pairing is by fresh arrivals from the same publisher cycle, not strict hardware synchronization. A watchdog publishes stop commands if input becomes stale.

The controller and estimator are in control.py, the map and A* are in navigation.py, and their state transitions are coordinated by navigator.py. The solver modules never import the engine or its hidden maze.

## Validation plan

The regression tests exercise wall-distance signs, the acceptable band, opening handling, PID saturation, physical centring from both sides, A* detours, reciprocal mapping, and collision/odometry disagreement.

The maze runner launches ten worker processes behind a common start barrier. Every worker uses the actual VirtualMouse physics and raycasts with a distinct generated maze. ROS mode uses actual publishers, subscriptions, and the production StudentSolver callbacks, isolated by ROS domain. Simulated timestamps accelerate the run; optional jitter and delayed command application model frame timing.

Only the test harness reads the engine's true pose and maze. It compares these with solver estimates, checks every mapped edge, counts collisions, and requires physical goal arrival plus solver completion. It logs reported speed separately from speed measured from actual displacement. JSONL traces and summaries are saved under artifacts/maze-tests and are excluded from Git. See docs/attempt2-validation.md for the verified run.
