"""Pose action and atomic speed-limit service sharing node parameters."""
import math
import threading
import time
from typing import Any
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import SetParametersResult
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.parameter import Parameter
from turtlesim.msg import Pose
from turtle_lab_interfaces.action import GoToPose
from turtle_lab_interfaces.srv import SetSpeedLimits
from .common import normalize, pause, run
from .controller import ProportionalController

class GoToPoseServer(Node):
    """Own the velocity publisher and dynamically adjustable control limits."""
    POSE_TIMEOUT = 2.0
    GOAL_TIMEOUT = 300.0

    def __init__(self) -> None:
        super().__init__('go_to_pose_server')
        self.group = ReentrantCallbackGroup()
        self.lock = threading.Lock()
        self.busy = False
        self.pose: Pose | None = None
        self.pose_time = 0.0
        for name in ('max_linear_speed', 'max_angular_speed'):
            value = self.declare_parameter(name, 2.0).value
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f'{name} must be finite and > 0')
        self.add_on_set_parameters_callback(self.validate_parameters)
        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.create_subscription(Pose, '/turtle1/pose', self.on_pose, 10,
                                 callback_group=self.group)
        self.create_service(SetSpeedLimits, 'set_speed_limits', self.set_limits,
                            callback_group=self.group)
        self.server = ActionServer(self, GoToPose, 'go_to_pose', self.execute,
                                   goal_callback=self.accept,
                                   cancel_callback=lambda _: CancelResponse.ACCEPT,
                                   callback_group=self.group)

    def on_pose(self, pose: Pose) -> None:
        """Store the latest immutable pose snapshot and receipt time."""
        self.pose = pose
        self.pose_time = time.monotonic()

    def validate_parameters(self, parameters: list[Parameter]) -> SetParametersResult:
        """Reject invalid values from either parameter entry point."""
        for parameter in parameters:
            if parameter.name in ('max_linear_speed', 'max_angular_speed'):
                if (parameter.type_ != Parameter.Type.DOUBLE or
                        not math.isfinite(parameter.value) or parameter.value <= 0):
                    return SetParametersResult(successful=False,
                                               reason='Speed limits must be finite doubles > 0')
        return SetParametersResult(successful=True)

    def set_limits(self, request: Any, response: Any) -> Any:
        """Validate and change both parameters atomically."""
        result = self.set_parameters_atomically([
            Parameter('max_linear_speed', value=request.max_linear_speed),
            Parameter('max_angular_speed', value=request.max_angular_speed)])
        response.success = result.successful
        response.message = 'Speed limits updated' if result.successful else result.reason
        return response

    def accept(self, goal: Any) -> GoalResponse:
        """Reject invalid coordinates and concurrent motion requests."""
        target = goal.target
        if not (all(math.isfinite(v) for v in (target.x, target.y, target.yaw))
                and 0 < target.x < 11 and 0 < target.y < 11):
            self.get_logger().error('Rejected goal: finite target required; 0 < x,y < 11')
            return GoalResponse.REJECT
        with self.lock:
            if self.busy:
                self.get_logger().error('Rejected goal: another pose goal is active')
                return GoalResponse.REJECT
            self.busy = True
        self.get_logger().info(f'Accepted go_to_pose: ({target.x}, {target.y}, {target.yaw})')
        return GoalResponse.ACCEPT

    async def execute(self, handle: Any) -> GoToPose.Result:
        """Run a 50 Hz controller while allowing cancellation and parameter calls."""
        target = handle.request.target
        result = GoToPose.Result(success=False, message='Motion aborted')
        controller = ProportionalController()
        started = time.monotonic()
        try:
            while True:
                if self.stopping:
                    handle.abort()
                    result.message = 'Pose server shutting down'
                    break
                if handle.is_cancel_requested:
                    handle.canceled()
                    result.message = 'Pose goal canceled'
                    break
                now = time.monotonic()
                if now - started > self.GOAL_TIMEOUT:
                    raise TimeoutError('Goal exceeded 300 seconds')
                pose = self.pose
                if pose is None:
                    if now - started > self.POSE_TIMEOUT:
                        raise TimeoutError('No turtle pose received')
                    await pause(self)
                    continue
                if now - self.pose_time > self.POSE_TIMEOUT:
                    raise TimeoutError('Turtle pose is stale')
                linear, angular, distance, angle, done = controller.velocity(
                    pose.x, pose.y, pose.theta, target.x, target.y, target.yaw)
                result.final_distance_error = distance
                result.final_yaw_error = abs(normalize(target.yaw - pose.theta))
                if done:
                    handle.succeed()
                    result.success = True
                    result.message = 'Target pose reached'
                    break
                max_linear = self.get_parameter('max_linear_speed').value
                max_angular = self.get_parameter('max_angular_speed').value
                twist = Twist()
                twist.linear.x = max(-max_linear, min(max_linear, linear))
                twist.angular.z = max(-max_angular, min(max_angular, angular))
                self.publisher.publish(twist)
                handle.publish_feedback(GoToPose.Feedback(
                    distance_remaining=distance, angle_remaining=angle))
                await pause(self)
        except Exception as error:
            result.message = str(error)
            self.get_logger().error(result.message)
            if handle.is_active:
                handle.abort()
        finally:
            self.publisher.publish(Twist())
            with self.lock:
                self.busy = False
        return result

def main(args: list[str] | None = None) -> None:
    """Executable entry point."""
    run(GoToPoseServer, args)
