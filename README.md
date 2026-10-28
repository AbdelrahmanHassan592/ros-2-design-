# ros-2-design-
# Lab 1: ROS 2, Turtlesim, Actions & Services

Team members: **Abdelrahman Hassan Gaber - 2204044, Hamdy adel elmohamdy - 2204172, ahmed mohamed ahmed ali - 2204297** 



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




