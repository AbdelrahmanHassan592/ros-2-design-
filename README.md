# Lab 1: ROS 2, Turtlesim, Actions & Services

Team members: **Abdelrahman Hassan Gaber - 2204044, Hamdy adel elmohamdy - 2204172, ahmed mohamed ahmed ali - 2204297**

This repository belongs at `~/turtle_ws/src`. The workspace's `build`, `install`, and
`log` directories live one level above the repository. No bags are committed.

| Package | Purpose |
| --- | --- |
| `turtle_lab_interfaces` | ament_cmake message, two actions, speed service |
| `turtle_lab_control` | ament_python pose controller, reset service, square coordinator, bag utility |
| `turtle_lab_bringup` | Installed launch files and YAML defaults |

## Prerequisites and build

Tested on Ubuntu 24.04, ROS 2 Jazzy, Python 3.12. Install ROS Jazzy using the
[official Ubuntu installation instructions](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html).
Then:

```bash
sudo apt update
sudo apt install ros-jazzy-desktop ros-jazzy-turtlesim ros-jazzy-rosbag2 \
  python3-colcon-common-extensions python3-rosdep python3-pytest python3-yaml
# Run sudo rosdep init only if rosdep has never been initialized.
rosdep update
mkdir -p ~/turtle_ws/src
# Clone this repository INTO ~/turtle_ws/src (not into an additional subfolder).
source /opt/ros/jazzy/setup.bash
cd ~/turtle_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --event-handlers console_direct+
source install/setup.bash
colcon test --event-handlers console_direct+
colcon test-result --verbose
```

In **each new terminal**, source `/opt/ros/jazzy/setup.bash` and
`~/turtle_ws/install/setup.bash`. For a headless machine prefix launch commands with
`QT_QPA_PLATFORM=offscreen`; the simulator still publishes poses and executes services.
Do not start a second turtlesim or another lab instance in the same ROS domain.

## R1 — Interfaces

```bash
ros2 interface list | grep turtle_lab
ros2 interface show turtle_lab_interfaces/msg/TargetPose
ros2 interface show turtle_lab_interfaces/action/GoToPose
ros2 interface show turtle_lab_interfaces/action/DrawSquare
ros2 interface show turtle_lab_interfaces/srv/SetSpeedLimits
```

## R2 — Complete launch and pose action

Terminal A:

```bash
ros2 launch turtle_lab_bringup lab1.launch.py
```

Terminal B (sequential goals; every command waits for its result):

```bash
ros2 action send_goal --feedback /go_to_pose turtle_lab_interfaces/action/GoToPose '{target: {x: 7.0, y: 5.544445, yaw: 0.7}}'
ros2 action send_goal --feedback /go_to_pose turtle_lab_interfaces/action/GoToPose '{target: {x: 4.0, y: 5.544445, yaw: -0.4}}'
ros2 action send_goal --feedback /go_to_pose turtle_lab_interfaces/action/GoToPose '{target: {x: 4.0, y: 8.0, yaw: 2.6}}'
ros2 action send_goal --feedback /go_to_pose turtle_lab_interfaces/action/GoToPose '{target: {x: 4.0, y: 3.0, yaw: -2.0}}'
ros2 action send_goal --feedback /go_to_pose turtle_lab_interfaces/action/GoToPose '{target: {x: 7.0, y: 3.0, yaw: 3.14159265}}'
ros2 topic echo /turtle1/pose --once
# Rejected; explanation appears in the server log (the ROS action rejection has no message field):
ros2 action send_goal /go_to_pose turtle_lab_interfaces/action/GoToPose '{target: {x: 11.0, y: 5.0, yaw: 0.0}}'
```

The controller rotates toward the target, drives with proportional distance and
heading correction, then rotates to final yaw. Angles use `atan2(sin, cos)`.
Completion requires distance ≤ 0.1 and final yaw error ≤ 0.05 rad. `angle_remaining`
is heading error during travel and final yaw error during alignment; result errors
always refer to the target pose. Commands run at approximately 50 Hz. Each iteration
reads and applies both speed parameters. Every exit publishes zero velocity.
A missing/stale pose aborts after 2 seconds; goals have a 300-second deadline.
Very small valid speed limits can therefore cause a timeout.

## R3 — Dynamic ROS parameters

```bash
ros2 param list /go_to_pose_server
ros2 param get /go_to_pose_server max_linear_speed
ros2 param get /go_to_pose_server max_angular_speed
ros2 param set /go_to_pose_server max_linear_speed 0.4
ros2 param set /go_to_pose_server max_angular_speed 0.5
# Both rejected; existing values remain:
ros2 param set /go_to_pose_server max_linear_speed 0.0
ros2 param set /go_to_pose_server max_angular_speed -1.0
```

Use double literals (`2.0`, not `2`). Change parameters in another terminal while a
long pose goal is moving to see the limit apply immediately. Restore both to 2.0
before comparing normal-speed squares.

## R4 — Atomic speed-limit service

```bash
ros2 service call /set_speed_limits turtle_lab_interfaces/srv/SetSpeedLimits '{max_linear_speed: 2.0, max_angular_speed: 2.0}'
ros2 service call /set_speed_limits turtle_lab_interfaces/srv/SetSpeedLimits '{max_linear_speed: 0.0, max_angular_speed: 2.0}'
ros2 service call /set_speed_limits turtle_lab_interfaces/srv/SetSpeedLimits '{max_linear_speed: 2.0, max_angular_speed: -1.0}'
ros2 param get /go_to_pose_server max_linear_speed
ros2 param get /go_to_pose_server max_angular_speed
```

Invalid/nonfinite values change neither parameter. The service uses
`set_parameters_atomically`, so native parameter calls and the service share the
same validation and storage.

## R5 — Reset without a teleport line

Wait for motion to finish or cancel it before resetting. Reset owns no velocity
publisher and deliberately uses only the three specified turtlesim services.

```bash
ros2 service call /reset_turtle std_srvs/srv/Trigger '{}'
ros2 service call /reset_turtle std_srvs/srv/Trigger '{}'
ros2 service call /reset_turtle std_srvs/srv/Trigger '{}'
ros2 topic echo /turtle1/pose --once
```

Expected: `(5.544445, 5.544445, 0.0)` and a clean canvas. The awaited order is pen
off → teleport → clear → pen on `(179,184,255,width=3)`. Pen restoration is attempted
also after an intermediate failure. Concurrent reset requests receive a clear busy
response. Clients have 10-second deadlines. Services use asynchronous clients and
reentrant callback groups; no nested spin or blocking wait holds an executor thread.

## R6 — Square twice, validation and cancellation

```bash
ros2 action send_goal --feedback /draw_square turtle_lab_interfaces/action/DrawSquare '{side_length: 2.0}'
ros2 action send_goal --feedback /draw_square turtle_lab_interfaces/action/DrawSquare '{side_length: 2.0}'
# Rejected:
ros2 action send_goal /draw_square turtle_lab_interfaces/action/DrawSquare '{side_length: 0.0}'
ros2 action send_goal /draw_square turtle_lab_interfaces/action/DrawSquare '{side_length: 6.0}'
```

The side must be finite and `0 < side_length < 5.455555`. Each accepted square
resets first and sends exactly four goals, waiting for each result:

| Side | x | y | Final yaw |
| --- | --- | --- | --- |
| 1 | x0+s | y0 | π/2 |
| 2 | x0+s | y0+s | π |
| 3 | x0 | y0+s | −π/2 |
| 4 | x0 | y0 | 0 |

`x0=y0=5.544445`. Logs show `Sending side N/4` and four corresponding
`Accepted go_to_pose` entries per completed square. Nominal sides are equal and
corners differ by 90°; actual traces vary within controller tolerances.
A failed child aborts the parent with a reason and completed-side count.
Child results have a 310-second deadline.
Only one goal per action server may be active. Do not send independent pose goals
or reset requests during a square.

The reproducible integration check sends real ROS cancellation requests for both
actions, validates every final pose, compares fast/slow timing, checks invalid goals,
checks live parameter updates, repeats squares and checks stopping commands:

```bash
python3 ~/turtle_ws/src/turtle_lab_control/test/integration.py
```

Run it with the lab idle. Ctrl+C in `ros2 action send_goal` only closes the client;
it is not a reliable action-cancellation test. Use the integration check instead.

## R7 — YAML and standalone launch

Stop Terminal A with Ctrl+C. In a fresh terminal, source the workspace and run only:

```bash
source /opt/ros/jazzy/setup.bash
source ~/turtle_ws/install/setup.bash
ros2 launch turtle_lab_bringup lab1.launch.py
```

In another sourced terminal:

```bash
ros2 node list
ros2 param get /go_to_pose_server max_linear_speed
ros2 param get /go_to_pose_server max_angular_speed
```

Expected nodes: `/turtlesim`, `/go_to_pose_server`, `/reset_service`,
`/square_action_server`. YAML defaults are 2.0/2.0. Stop that launch before trying:

```bash
ros2 launch turtle_lab_bringup lab1.launch.py max_linear_speed:=1.0 max_angular_speed:=1.5
```

For a fully automatic square, stop the previous launch and run:

```bash
ros2 launch turtle_lab_bringup lab1_square.launch.py side_length:=2.0
```

This includes the complete lab, then runs `ros2 action send_goal --feedback` after
3 seconds. The square server waits for its dependencies, so service discovery can
finish without manual startup. The lab keeps running after the square completes.

## R8 — Clean rebuild

Stop all lab processes. Open a fresh terminal **without sourcing the old overlay**:

```bash
source /opt/ros/jazzy/setup.bash
cd ~/turtle_ws
rm -rf build install log
colcon build --event-handlers console_direct+
source install/setup.bash
colcon test --event-handlers console_direct+
colcon test-result --verbose
ros2 launch turtle_lab_bringup lab1_square.launch.py
```

The repository itself contains no build artifacts; launch and config files are
installed through `setup.py`, rather than located using source-tree paths.

## Bonus 1 — Design patterns

**Strategy:** `ProportionalController.velocity` computes commands independently of
ROS transport. The action creates a fresh strategy for each goal, making it
replaceable without changing subscriptions, parameters or action lifecycle. Pure
controller tests exercise directions and transitions.

The strategy has an explicit `Phase` enum for rotate/drive/final-yaw. Three state
classes or a factory would add indirection without useful alternate implementations,
so they are intentionally omitted. Square composition uses a ROS action client
rather than duplicating the pose controller. Nodes run with four-thread executors,
reentrant callback groups, and coroutine waits that yield to ROS timers.

Reference: the [rclpy action server implementation](https://github.com/ros2/rclpy/blob/jazzy/rclpy/rclpy/action/server.py)
supports coroutine execute callbacks and explicit goal/cancel callbacks.

## Bonus 2 — Bag recording and playback

With the lab idle and running:

```bash
ros2 run turtle_lab_control bag_demo record ~/turtle_ws/square_bag
ros2 bag info ~/turtle_ws/square_bag
```

Choose a nonexistent output path. The utility records `/turtle1/cmd_vel` and
`/turtle1/pose`, triggers a complete square, then gracefully closes the recorder.
Stop the **entire lab launch**. Now run:

```bash
ros2 run turtle_lab_control bag_demo replay ~/turtle_ws/square_bag
```

Replay starts only turtlesim, uses native services to reset with the pen off, then
plays only `/turtle1/cmd_vel`. Recorded poses remain available for analysis; playing
them into `/turtle1/pose` would mix recorded state with the simulator's live state.
Expect approximately the same square and return near the start. Replay is open
loop: OS timing and simulator update timing can introduce differences. Inspect the
canvas, then Ctrl+C to stop the replay utility and its turtlesim.

Equivalent commands, if turtlesim is already the **only** running node:

```bash
ros2 service call /turtle1/set_pen turtlesim/srv/SetPen '{"off": 1}'
ros2 service call /turtle1/teleport_absolute turtlesim/srv/TeleportAbsolute '{x: 5.544445, y: 5.544445, theta: 0.0}'
ros2 service call /clear std_srvs/srv/Empty '{}'
ros2 service call /turtle1/set_pen turtlesim/srv/SetPen '{r: 179, g: 184, b: 255, width: 3, "off": 0}'
ros2 bag play ~/turtle_ws/square_bag --topics /turtle1/cmd_vel
```

## Validation record

Executed on the supplied machine using Qt's offscreen platform, followed by an
actual X11 GUI run and screenshot inspection:

| Check | Result |
| --- | --- |
| Interfaces | All four listed and displayed with `ros2 interface show` |
| Build | Three packages; clean build with no build warnings |
| Automated tests | 14 passed: controller geometry, reset ordering/failure recovery, installed launch assets |
| Native parameters and service | Valid changes applied; zero/negative/nonfinite rejected; invalid atomic requests left both values unchanged |
| Pose actions | Five directions via CLI `--feedback` and live Python assertions; distance ≤ 0.1, yaw ≤ 0.05 rad |
| Speed changes | Same move: 2.13 s at limit 2.0 versus 4.99 s at limit 0.4; live change and command clamps checked |
| Reset | Repeated after drawing and at start; position/yaw checked; clean GUI canvas inspected; unit tests verify pen-off precedes teleport and pen restoration follows failures |
| Squares | Twice consecutively; sides_completed=4, all feedback side numbers, exactly four sent/accepted child goals per run |
| Cancellation | Pose and square canceled; final velocity zero; overlapping pose goal rejected |
| Launch | All four nodes start from installed files; YAML 2.0/2.0 and overrides 1.0/1.5 checked; TimerAction square completed |
| Rebuild | Build/install/log removed and rebuilt; separate source copy also built/tested successfully |
| GUI | Visible square inspected; exact pen RGB (179,184,255) and 3-pixel straight-section width confirmed from screenshot pixels |
| Bag | Recorded 1,068 poses and 471 velocity messages (136 KiB); replayed velocity only with turtlesim and no control nodes |

The recorded square had four distinct translational intervals, with measured
headings changing by approximately 87.25°, 89.90° and 89.99°. Sampling and the
specified position/yaw tolerances account for deviations from the nominal square;
this is not an exact geometric drawing. Recorded final pose was
`(5.54945, 5.64188, -0.04758)`; one open-loop replay ended at
`(5.53557, 5.66355, -0.06447)`, approximately 2.6 cm and 0.017 rad away.
The temporary bag is at `/tmp/turtle-lab-square-bag`, outside Git.

**Limits of verification:** the square and blank canvas after reset were visually
inspected, but individual transient teleport frames were not captured. Pen-off
ordering was verified by the reset protocol tests. Replay is approximate rather
than bit-for-bit deterministic. GUI screenshots are available at
`/tmp/lab-square.png` and `/tmp/lab-reset.png`; they are outside Git.

Shutdown testing caught a teardown race during active actions. Nodes now handle
signals in the main loop, stop goals and drain callbacks before destroying ROS
entities. Idle shutdown and shutdown during pose/square motion were exercised.
