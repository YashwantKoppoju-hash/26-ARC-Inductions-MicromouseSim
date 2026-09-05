# ARC Micromouse Simulator — Induction Task

Welcome to the induction program for our club. This repository will guide you through the Micromouse task, helping you get started with Docker, ROS 2, and basic pathfinding algorithms. 

Your objective is to write a maze-solving algorithm inside `student_agent/solver.py` to navigate a virtual robot through an unknown maze and park it inside the green 2x2 goal zone. 

## Attempt #2

This branch implements online A* with PID wall centring. Start a fresh simulator,
then run `python3 student_agent/solver.py` inside the container. The solver assumes
the initial position (1.5, 1.5), facing north. Restart the solver together with the
simulator for another run. Random layouts are enabled.

For a fresh live run from PowerShell:

```powershell
docker compose up -d --force-recreate micromouse_simulator
docker exec micromouse_simulator bash -lc "source /opt/ros/humble/setup.bash && python3 student_agent/solver.py"
```

Mapping uses reciprocal unknown/open/wall edges. Successfully traversed edges
remain open in this static maze; contradictory sensor observations are counted
in `map_conflicts`. Observations require cardinal alignment and a ray crossing
away from cell corners, with an uncertainty band around wall detections.
The planner uses heap-based A* with unit edge costs and Manhattan distance;
regressions compare its path lengths against independent breadth-first search.

The ROS adapter is in `solver.py`; shared dataclasses are in `state.py`, wall PID
and pose estimation in `control.py`, map/A* in `navigation.py`, and movement
coordination in `navigator.py`. This branch deliberately uses multiple modules.
The simulator engine is unchanged; its reported speed can remain nonzero when
a wall prevents movement.

Legacy isolated regressions (not acceptance tests) can be run inside the ROS container with
`python3 -m unittest tests.test_navigation -v`.
The legacy synchronous test runner is
`python3 -m tests.run_mazes --workers 10 --ros --jitter --delay-frames 1`.
The headless tests use the actual engine physics and simulated time; each ROS
worker has an isolated domain. This runner waits for commands before advancing
physics and does not validate live timing. Acceptance tests must use the actual
rendered simulator with normal ROS scheduling. Traces compare engine truth with the solver's
estimate. See [Design Spec #2](Design%20Spec%20%232.md) and
[validation results](docs/attempt2-validation.md).

Overshoot and front clearance below 0.3 trigger `RECOVER_BRAKE`. Recovery uses
slow motion toward the estimated occupied cell center, with a bounded 0.12-unit
reverse toward the cell interior when front clearance is low, then cardinal
turns and forward corrections for the remaining axes. Mapping pauses until
the mouse stops within 0.06 of center. Recovery times out after 45 seconds if
clearance cannot be established; invalid localization still requires a reset.

Sensor gaps over 0.25 seconds now enter `TIMING_PAUSE`, command zero motion,
and suspend odometry/map updates. Queued scans older than 0.25 seconds are
discarded. Five fresh stationary pairs are required before matching wall
planes on both axes and entering recentering recovery. Displacement uncertainty
above 0.35 cells or heading uncertainty above 0.08 radians keeps the mouse
paused for relocalization/reset. Logs include scan age, interval, gap count,
and uncertainty. This is intentionally not guaranteed recovery from every gap.

Live validation of timing recovery used the normal rendered simulator on port
8081. Three gap recoveries resumed motion. A 350 ms SIGSTOP/SIGCONT interruption
of the solver while the engine continued running was also exercised; the run
subsequently remained paused with position uncertainty above the allowed bound.
This validation did not establish an end-to-end goal completion.

---

## Contents
1. [How to Submit Your Work](#how-to-submit-your-work)
2. [Prerequisites](#prerequisites)
3. [Your Task](#your-task)
4. [The 30-Point System](#the-30-point-system)
5. [Installation & Setup](#installation--setup)
6. [How to Control the Mouse (ROS 2 API)](#how-to-control-the-mouse-ros-2-api)
7. [Map Configuration](#map-configuration)
8. [Troubleshooting](#troubleshooting)

---

## How to Submit Your Work
1. **Fork this repository** into your own GitHub account.
2. Clone **your forked repository** to your local machine.
3. Complete the task by writing your solver logic inside `student_agent/solver.py`.
4. Run `git add .` followed by `git commit -m <PR Title>` to commit your changes.
5. Run `git push` to push your changes to GitHub.
6. Submit a **Pull Request (PR)** to the main repository.
   * **PR Title format:** `NAME [ID_NUMBER]` (Example: `Archisman Das [2026B3PS0478H]`).
   * **PR Description format:** Must include your Full Name, ID Number, and Institute Email.
7. Wait for review and feedback!

---

## Prerequisites

You are not expected to have any prior robotics software installed on your machine. Everything runs inside an isolated container. Before starting, please ensure you have the following installed:
* **Git**
* **Docker** (Windows users: Install Docker Desktop and ensure WSL2 is enabled).
* **Docker Compose**
* Basic knowledge of the Linux terminal.

---

## Your Task
1. **Analyze the Example:** We have provided a baseline script in `student_agent/solver.py`. You can run this file inside the container to see how the robot interacts with the maze and the ROS 2 API.
2. **Write Your Solver:** Your actual algorithm must be written inside `student_agent/solver.py`. **You only need to edit this single file—do not modify the simulator engine, maze generator, or any other files.**
3. **Solve the Maze:** Your goal is to write a robust algorithm in the `scan_callback` function of `solver.py` that can consistently navigate the virtual robot into the green 2x2 goal zone on *any* random map layout. 

---

## The 30-Point System

At the top of your `student_agent/solver.py` script, you must set your robot's physical stats. 

You have a strict budget of **30 points** to distribute among four physical traits. These four must add up to exactly 30:

```python
TOP_SPEED = 8
ACCELARATION = 7
TURN_SPEED = 5
SENSOR_RANGE = 10
```

If your total exceeds 30, the simulation engine will crash immediately. You must balance these stats based on how your algorithm behaves—a fast bot with terrible sensors might crash into walls, while a bot with maximum sensors might be too slow to get a competitive track time.

---

## Installation & Setup

**Clone the Repository:**
Make sure you are cloning your own fork.
```bash
git clone <your-fork-url>
cd 26-ARC-Inductions-MicromouseSim
```

**Start the Simulator Engine:**
```bash
docker-compose up -d --build
```
*(Apple Silicon Mac / Raspberry Pi users: prefix the command with `DOCKER_DEFAULT_PLATFORM=linux/arm64`)*

**View the UI:**
Open a web browser and go to http://localhost:8080. You will access a Linux desktop in your browser using noVNC. You should see the live Pygame simulation waiting for commands.

**Run Your Solver:**
Leave the simulator running in the background. Open a second terminal window on your host machine and drop into the container to execute your code:
```bash
docker exec -it micromouse_simulator bash
python3 student_agent/solver.py
```

---

## How to Control the Mouse (ROS 2 API)

We have already written the ROS 2 boilerplate for you in `student_agent/solver.py`! You just need to use the provided variables to read the sensors and set the movement speeds.

### 1. Reading Sensors
Inside your `scan_callback` function, the bot emits three raycasts at 20 Hz. The distances to the nearest walls are automatically extracted into these variables for you:
```python
d_left  = msg.ranges[0]   # Distance to left wall
d_front = msg.ranges[1]   # Distance to front wall
d_right = msg.ranges[2]   # Distance to right wall
```
*(Note: Larger values mean more open space.)*

### 2. Moving the Bot
A Twist message named `cmd` is already initialized in your callback. To move the bot, simply set its linear and angular speeds:
```python
cmd.linear.x  = 0.5   # Forward speed (+forward, -reverse)
cmd.angular.z = 1.0   # Turn rate (+left/CCW, -right/CW)
```
The template code handles publishing this command (`self.cmd_pub.publish(cmd)`) at the very end of the function automatically.

**Safety Feature:** If your solver crashes or stops sending commands for 0.5 seconds, the mouse will automatically hit the brakes.

---

## Map Configuration

By default, the simulator loads a fixed map *[Seed 67 ;)]* so you can consistently test and tune your parameters. Once your algorithm works, you must test it against random layouts!

Open `simulator/maze_layouts.py` and change the configuration at the bottom of the file:
```python
# True = Uses the same map every time (good for testing/tuning).
# False = Generates a completely new random map on every run.
USE_FIXED_MAP = False
```

---

## Development Workflow

* The project folder is bind-mounted into the Docker container. This means you do not need to restart Docker or rebuild the image when you write code.
* Edit `student_agent/solver.py` using VS Code (or your preferred editor) on your host machine (Windows/Mac/Linux).
* Save the file.
* Stop (`Ctrl+C`) and restart the `python3 student_agent/solver.py` script in your second terminal.

*(Note: Only changes to the `Dockerfile` or `requirements.txt` require a full `docker-compose up -d --build`)*

---

## Troubleshooting

* **Black screen in browser:** Wait 5–10 seconds after `docker-compose up` for background services to start. If it stays black, check `docker logs micromouse_simulator`.
* **"Cannot connect to display :1":** The virtual display crashed. Run `docker-compose down && docker-compose up -d` for a clean restart.
* **Solver connects but no motion:** Ensure you don't have multiple ROS nodes conflicting. You can force isolation by running `export ROS_DOMAIN_ID=42` inside the container before running your solver.
* **Code changes not reflecting:** Ensure you are restarting the `solver.py` script in your second terminal after saving your changes.
