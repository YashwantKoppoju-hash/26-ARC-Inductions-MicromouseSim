"""Ten concurrent, isolated runs of the real engine with oracle-only telemetry.

Run inside the ROS container: python3 -m tests.run_mazes --workers 10
Physics runs at 60 Hz simulated time, control at 20 Hz; rendering is omitted.
Only SensorValues/MotionState cross into the production navigation code.
"""

import argparse
import hashlib
from concurrent.futures import ProcessPoolExecutor, as_completed
from datetime import datetime, timezone
import json
import math
import multiprocessing
import os
from pathlib import Path
import random
import sys
import time


def run(seed, output, barrier, max_seconds, jitter=False, ros=False, domain_id=110, delay_frames=0):
    root = Path(__file__).resolve().parents[1]
    sys.path.insert(0, str(root / 'simulator'))
    import sim_engine as engine
    import maze_layouts
    from student_agent.solver import Navigator, TOP_SPEED, ACCELARATION, TURN_SPEED, SENSOR_RANGE
    from student_agent.state import Command, MotionState, SensorValues, Cell, Direction, wrap_angle

    engine.MAZE_GRID = maze_layouts.generate_maze(seed)
    config = engine.validate_and_load_constraints()
    mouse = engine.VirtualMouse(config)
    navigator = Navigator(TOP_SPEED * 0.2, ACCELARATION * 0.1, TURN_SPEED * 0.15, SENSOR_RANGE * 0.4)
    bridge = None
    if ros:
        from tests.ros_bridge import RosBridge
        bridge = RosBridge(engine, mouse, domain_id)
        navigator = bridge.solver.navigator
    rng = random.Random(seed)
    collisions = 0
    reached_at = None
    max_error = max_angle = 0.0
    duration = 0.0
    step_times = []
    command = Command()
    previous_targets = (0.0, 0.0)
    final = None
    barrier.wait(timeout=90)
    wall_start = time.time()
    logfile = Path(output) / f'maze-{seed}.jsonl'
    with logfile.open('w', buffering=65536) as log:
        while duration < max_seconds:
            interval = 0.0
            before_x, before_y = mouse.x, mouse.y
            requested = (mouse.target_linear, mouse.target_angular) if bridge else (command.linear, command.angular)
            for frame in range(3):
                dt = rng.uniform(0.014, 0.020) if jitter else 1 / 60
                interval += dt
                if reached_at is None:
                    old_x, old_y = mouse.x, mouse.y
                    # Model the GUI loop processing cmd_vel on its next spin.
                    linear, angular = previous_targets if frame < delay_frames else requested
                    mouse.set_targets(max(-config['max_speed'], min(config['max_speed'], linear)),
                                      max(-config['max_turn_rate'], min(config['max_turn_rate'], angular)))
                    mouse.update(dt)
                    if mouse.v_linear > 1e-4 and mouse.x == old_x and mouse.y == old_y:
                        collisions += 1
                    if 7 <= mouse.x <= 9 and 7 <= mouse.y <= 9:
                        reached_at = duration + interval
                        mouse.set_targets(0.0, 0.0)
                # Match sim_engine.main: goal freezes physics, but ROS keeps publishing.
            duration += interval
            previous_targets = requested
            sensors = SensorValues(*mouse.calculate_ui_raycasts())
            motion = MotionState(mouse.v_linear, mouse.v_angular)
            begin = time.perf_counter_ns()
            command = navigator.step(sensors, motion, interval) if bridge is None else bridge.step(interval)
            step_times.append((time.perf_counter_ns() - begin) / 1e6)
            error = math.hypot(navigator.pose.x - mouse.x, navigator.pose.y - mouse.y)
            angle_error = abs(wrap_angle(navigator.pose.theta - mouse.heading))
            max_error, max_angle = max(error, max_error), max(angle_error, max_angle)
            ground_speed = math.hypot(mouse.x - before_x, mouse.y - before_y) / interval
            record = dict(seed=seed, sim_time=duration, engine_pose=[mouse.x, mouse.y, mouse.heading],
                          estimated_pose=[navigator.pose.x, navigator.pose.y, navigator.pose.theta],
                          position_error=error, heading_error=angle_error,
                          reported_velocity=[mouse.v_linear, mouse.v_angular], ground_speed=ground_speed,
                          scan=[sensors.left, sensors.front, sensors.right], collisions=collisions,
                          engine_goal=reached_at is not None, **navigator.status())
            log.write(json.dumps(record) + '\n')
            final = record
            if navigator.phase in ('FAILED', 'COMPLETE'):
                break
            if reached_at is not None and duration - reached_at > 10:
                break
    mismatches = []
    for r in range(16):
        for c in range(16):
            cell = Cell(r, c)
            for direction in Direction:
                observed = navigator.maze.edge(cell, direction)
                if observed is None:
                    continue
                dr, dc = direction.delta
                actual = bool(engine.MAZE_GRID[2 * r + 1 + dr, 2 * c + 1 + dc])
                if observed != actual:
                    mismatches.append([r, c, direction.name, observed, actual])
    step_times.sort()
    passed = (reached_at is not None and navigator.phase == 'COMPLETE' and collisions == 0
              and max_error < 0.10 and max_angle < 0.12 and not mismatches)
    result = dict(seed=seed, pid=os.getpid(), passed=passed, phase=navigator.phase,
                  reason=navigator.reason, physical_goal=reached_at is not None,
                  simulated_seconds=duration, reached_at=reached_at, collisions=collisions,
                  max_position_error=max_error, max_heading_error=max_angle,
                  moves=navigator.moves, astar_searches=navigator.planner.searches,
                  step_ms_p99=step_times[int(0.99 * (len(step_times) - 1))],
                  map_mismatches=mismatches, wall_started=wall_start, wall_finished=time.time(),
                  log=logfile.name, final=final)
    (Path(output) / f'maze-{seed}-result.json').write_text(json.dumps(result, indent=2) + '\n')
    if bridge is not None:
        bridge.close()
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--workers', type=int, default=10)
    parser.add_argument('--seeds', type=int, nargs='+')
    parser.add_argument('--max-seconds', type=float, default=3600)
    parser.add_argument('--jitter', action='store_true')
    parser.add_argument('--ros', action='store_true', help='Exercise real ROS publishers and solver callbacks')
    parser.add_argument('--delay-frames', type=int, choices=(0, 1, 2), default=0,
                        help='Extra 60 Hz frames before a command takes effect')
    parser.add_argument('--output')
    args = parser.parse_args()
    if not 1 <= args.workers <= 20:
        parser.error('Use between 1 and 20 workers')
    seeds = args.seeds or random.SystemRandom().sample(range(1, 2**31), args.workers)
    if len(seeds) != args.workers:
        parser.error('Supply one seed per worker so all runs start together')
    if len(set(seeds)) != len(seeds):
        parser.error('Use distinct seeds so each worker has its own maze and log')
    output = Path(args.output or ('artifacts/maze-tests/' + datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')))
    output.mkdir(parents=True, exist_ok=True)
    root = Path(__file__).resolve().parents[1]
    sources = list((root / 'student_agent').glob('*.py')) + list((root / 'tests').glob('*.py'))
    sources += [root / 'simulator/sim_engine.py', root / 'simulator/maze_layouts.py']
    digests = {str(path.relative_to(root)): hashlib.sha256(path.read_bytes()).hexdigest() for path in sources}
    print(json.dumps(dict(seeds=seeds, workers=args.workers, ros=args.ros, jitter=args.jitter,
                          delay_frames=args.delay_frames, output=str(output))), flush=True)
    context = multiprocessing.get_context('spawn')
    results = []
    with context.Manager() as manager:
        barrier = manager.Barrier(args.workers)
        with ProcessPoolExecutor(max_workers=args.workers, mp_context=context) as pool:
            futures = [pool.submit(run, seed, str(output), barrier, args.max_seconds, args.jitter, args.ros, 110 + i, args.delay_frames)
                       for i, seed in enumerate(seeds)]
            for future in as_completed(futures):
                result = future.result()
                results.append(result)
                print(json.dumps({k: result[k] for k in ('seed', 'passed', 'phase', 'simulated_seconds',
                                                         'collisions', 'max_position_error', 'max_heading_error', 'reason')}), flush=True)
    overlap = min(r['wall_finished'] for r in results) - max(r['wall_started'] for r in results)
    unchanged = all(hashlib.sha256((root / path).read_bytes()).hexdigest() == digest
                    for path, digest in digests.items())
    summary = dict(workers=args.workers, seeds=seeds, jitter=args.jitter, ros=args.ros, delay_frames=args.delay_frames,
                   source_sha256=digests, source_unchanged_during_run=unchanged,
                   all_workers_overlapped=overlap > 0, overlap_seconds=overlap,
                   passed=sum(r['passed'] for r in results), runs=results)
    (output / 'summary.json').write_text(json.dumps(summary, indent=2) + '\n')
    print(f"Result: {summary['passed']}/{len(results)} passed; concurrent={overlap > 0}; logs={output}", flush=True)
    return 0 if summary['passed'] == len(results) and overlap > 0 and unchanged else 1


if __name__ == '__main__':
    raise SystemExit(main())
