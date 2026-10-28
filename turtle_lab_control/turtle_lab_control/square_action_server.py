"""Compose four pose actions into a cancellable square action."""
import math
import threading
import time
from typing import Any
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from std_srvs.srv import Trigger
from turtle_lab_interfaces.action import DrawSquare, GoToPose
from turtle_lab_interfaces.msg import TargetPose
from .common import START, SERVICE_TIMEOUT, bounded, pause, run

class SquareActionServer(Node):
    """Reset, then send exactly four sequential corner goals."""
    def __init__(self) -> None:
        super().__init__('square_action_server')
        self.group = ReentrantCallbackGroup()
        self.lock = threading.Lock()
        self.busy = False
        self.reset_client = self.create_client(Trigger, 'reset_turtle', callback_group=self.group)
        self.pose_client = ActionClient(self, GoToPose, 'go_to_pose', callback_group=self.group)
        self.server = ActionServer(self, DrawSquare, 'draw_square', self.execute,
                                   goal_callback=self.accept,
                                   cancel_callback=lambda _: CancelResponse.ACCEPT,
                                   callback_group=self.group)

    def accept(self, goal: Any) -> GoalResponse:
        """Reserve a single square if all corners fit strictly in the window."""
        side = goal.side_length
        if not math.isfinite(side) or not 0 < side < 11 - START:
            self.get_logger().error('Rejected square: require 0 < side_length < 5.455555')
            return GoalResponse.REJECT
        with self.lock:
            if self.busy:
                self.get_logger().error('Rejected square: another square is active')
                return GoalResponse.REJECT
            self.busy = True
        return GoalResponse.ACCEPT

    async def execute(self, handle: Any) -> DrawSquare.Result:
        """Await reset and each child; forward cancellation before returning."""
        result = DrawSquare.Result(success=False, sides_completed=0)
        child = None
        try:
            deadline = time.monotonic() + SERVICE_TIMEOUT
            while not (self.reset_client.service_is_ready() and self.pose_client.server_is_ready()):
                if handle.is_cancel_requested:
                    handle.canceled()
                    result.message = 'Square canceled before reset'
                    return result
                if time.monotonic() > deadline:
                    raise TimeoutError('Reset service or pose action unavailable')
                await pause(self)
            reset = await bounded(self, self.reset_client.call_async(Trigger.Request()))
            if not reset.success:
                raise RuntimeError(f'Reset failed: {reset.message}')
            side = handle.request.side_length
            corners = [(START + side, START, math.pi / 2),
                       (START + side, START + side, math.pi),
                       (START, START + side, -math.pi / 2), (START, START, 0.0)]
            for number, (x, y, yaw) in enumerate(corners, 1):
                if handle.is_cancel_requested:
                    handle.canceled()
                    result.message = 'Square canceled'
                    return result
                def feedback(message: Any, current: int = number) -> None:
                    if handle.is_active and not handle.is_cancel_requested:
                        handle.publish_feedback(DrawSquare.Feedback(
                            current_side=current,
                            distance_remaining=message.feedback.distance_remaining))
                self.get_logger().info(f'Sending side {number}/4: ({x:.6f}, {y:.6f}, {yaw:.6f})')
                child = await bounded(self, self.pose_client.send_goal_async(
                    GoToPose.Goal(target=TargetPose(x=x, y=y, yaw=yaw)),
                    feedback_callback=feedback))
                if not child.accepted:
                    child = None
                    raise RuntimeError(f'Side {number} pose goal rejected')
                pending = child.get_result_async()
                cancel_sent = False
                while not pending.done():
                    if handle.is_cancel_requested and not cancel_sent:
                        await bounded(self, child.cancel_goal_async())
                        cancel_sent = True
                    await pause(self)
                outcome = pending.result()
                child = None
                if handle.is_cancel_requested:
                    handle.canceled()
                    result.message = 'Square canceled; active pose goal stopped'
                    return result
                if outcome.status != GoalStatus.STATUS_SUCCEEDED or not outcome.result.success:
                    raise RuntimeError(f'Side {number} failed: {outcome.result.message}')
                result.sides_completed = number
            handle.succeed()
            result.success = True
            result.message = 'Four sides completed'
        except Exception as error:
            if child is not None and child.accepted:
                await bounded(self, child.cancel_goal_async())
            result.message = str(error)
            self.get_logger().error(result.message)
            if handle.is_active:
                handle.abort()
        finally:
            with self.lock:
                self.busy = False
        return result

def main(args: list[str] | None = None) -> None:
    """Executable entry point."""
    run(SquareActionServer, args)
