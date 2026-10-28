"""Shared constants, async waits and executor lifecycle."""
import math
import time
from typing import Any
import rclpy
from rclpy.executors import MultiThreadedExecutor, ExternalShutdownException
from rclpy.node import Node
from rclpy.task import Future

START = 5.544445
PERIOD = 0.02
POSITION_TOLERANCE = 0.1
YAW_TOLERANCE = 0.05
SERVICE_TIMEOUT = 10.0

def normalize(angle: float) -> float:
    """Wrap an angle to [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))

async def pause(node: Node, seconds: float = PERIOD) -> None:
    """Yield to the ROS executor without blocking a worker thread."""
    future = Future(executor=node.executor)
    def wake() -> None:
        if not future.done():
            future.set_result(None)
    timer = node.create_timer(seconds, wake)
    try:
        await future
    finally:
        node.destroy_timer(timer)

async def bounded(node: Node, future: Future, timeout: float = SERVICE_TIMEOUT) -> Any:
    """Await a ROS operation with a wall-clock deadline."""
    deadline = time.monotonic() + timeout
    while not future.done():
        if time.monotonic() >= deadline:
            future.cancel()
            raise TimeoutError('ROS operation timed out')
        await pause(node)
    return future.result()

def run(node_type: type[Node], args: list[str] | None = None) -> None:
    """Run callbacks concurrently and shut down cleanly."""
    rclpy.init(args=args)
    node = node_type()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
