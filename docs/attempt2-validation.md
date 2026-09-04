# Attempt #2 validation

Verified on 2026-09-04 using the ROS 2 Humble container. The final ten runs passed:
the actual engine entered the central goal in every run, every solver reported
COMPLETE, no physics steps collided, and every observed map edge agreed with
the hidden maze in the independent test oracle.

## What was exercised

- Ten isolated OS processes, each with a different generated maze and its own
  ROS domain (110–119), started behind a common barrier.
- The actual VirtualMouse.update collision/acceleration implementation and its
  exact raycasts from simulator/sim_engine.py. The engine file was not modified.
- The actual SimNode publishers, StudentSolver scan/velocity callbacks,
  and cmd_vel subscription over DDS.
- Physics timesteps varying from 0.014 to 0.020 seconds, three frames per scan,
  with a one-frame command-application delay. Simulated time runs faster than
  wall time; these are headless runs, not ten rendered Pygame windows.
- Only ranges and the engine's reported speed states entered the solver.
  Ground-truth pose and map data were read only by the test harness.
- Random layouts enabled by USE_FIXED_MAP = False. The seeds below were sampled
  randomly for the preceding batch and then replayed for the final audit.
- Source hashes saved in summary.json. Files stayed unchanged during the final
  run, and their hashes matched the working tree after testing.

All ten processes overlapped for 2.408 seconds.
The maximum Euclidean position-estimate error was
0.02561 world units
(one cell is one unit); maximum wrapped heading error was
0.03153 radians.
These are measured results for the listed mazes, not a guarantee for every
possible maze or hardware timing condition.

## Results

Moves counts fully completed cell-centre moves before the final goal-entry move.

| Seed | Simulated seconds | Moves | Maximum position error | Maximum heading error (rad) | Result |
| --- | ---: | ---: | ---: | ---: | --- |
| 66674236 | 66.71 | 13 | 0.02368 | 0.01839 | Pass |
| 632504728 | 236.70 | 48 | 0.00589 | 0.01743 | Pass |
| 890966393 | 292.03 | 59 | 0.01298 | 0.02525 | Pass |
| 1158887084 | 319.81 | 63 | 0.01040 | 0.02351 | Pass |
| 1344958183 | 313.62 | 66 | 0.00549 | 0.01587 | Pass |
| 1369217694 | 650.59 | 132 | 0.00810 | 0.01916 | Pass |
| 1704992577 | 549.65 | 115 | 0.02547 | 0.03153 | Pass |
| 1710414544 | 362.75 | 73 | 0.02561 | 0.02515 | Pass |
| 1747447621 | 349.72 | 72 | 0.00604 | 0.02128 | Pass |
| 1749868975 | 121.81 | 23 | 0.00652 | 0.02004 | Pass |

Pass conditions require engine-confirmed goal entry, solver completion, zero
collisions, no incorrect mapped edges, position error below 0.10 units, and
heading error below 0.12 radians. The test process exits nonzero on a failed
run, nonconcurrent execution, or a source change during the run.

Seven additional regression tests passed, covering PID direction and deadband,
anti-windup, openings and wall ends, invalid sensors, A* detours and reciprocal
walls, physical centring from either side of a corridor, and false velocity
reporting after a collision.

## Reproduce

Inside the ROS container, run:

```bash
python3 -m unittest tests.test_navigation -v
python3 -m tests.run_mazes --workers 10 --ros --jitter --delay-frames 1 \
  --seeds 66674236 1749868975 632504728 890966393 1344958183 \
          1158887084 1747447621 1710414544 1369217694 1704992577
```

Omit --seeds for ten newly sampled mazes. Each run records its seeds before it
starts. The original final report is at
artifacts/maze-tests/final-verified/summary.json, with one maze-<seed>.jsonl
trace and result JSON per worker. Generated traces are deliberately ignored by
Git; this report and the runner remain in the source tree.

## Reading the traces

Each 20 Hz trace record includes simulated time, engine_pose, estimated_pose,
position_error, wrapped heading_error, scan, reported_velocity, ground_speed,
command, phase, logical cell, target, centring error, collision count, and
engine_goal. Speed calculated from actual displacement is recorded separately
from the engine's internal speed.

For example, the final row for seed 66674236 has engine position approximately
(7.49465, 7.00275), estimate (7.49964, 7.02590), and engine_goal true. Reported
linear speed is still 0.55 while actual displacement speed is zero: the GUI
engine freezes physics on goal arrival while continuing to publish its stored
velocity. This demonstrates why those measurements must remain distinct.

The step_ms_p99 field in ROS mode includes the DDS round trip and executor
scheduling; it is not an isolated Python function benchmark.

## Boundaries of this result

A* replans against an incompletely observed maze, so first-run exploration is
not promised to be the globally shortest journey. The solver assumes a fresh
start at (1.5, 1.5), facing north. It stops with a diagnostic on unexpected
obstructions, stale/invalid measurements, or lost progress; it does not recover
arbitrary pose loss or reuse saved maps. Runtime diagnostics are emitted at
state changes and at most once a second otherwise.
