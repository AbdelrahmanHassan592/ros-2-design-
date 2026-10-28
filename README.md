# Lab 1: ROS 2, Turtlesim, Actions & Services

Team members: **Abdelrahman Hassan Gaber - 2204044,
Hamdy adel elmohamdy - 2204172,
ahmed mohamed ahmed ali elsharkawy- 2204297**



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
