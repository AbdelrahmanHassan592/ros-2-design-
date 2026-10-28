#!/usr/bin/env python3
"""Live integration checks; run with the installed lab already launched."""
import math
import time
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.srv import GetParameters, SetParameters
from std_srvs.srv import Trigger
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtle_lab_interfaces.action import GoToPose, DrawSquare
from turtle_lab_interfaces.msg import TargetPose
from turtle_lab_interfaces.srv import SetSpeedLimits

rclpy.init()
node = Node('lab_integration_test')
pose = None
commands = []
def on_pose(value):
    global pose
    pose = value
node.create_subscription(Pose, '/turtle1/pose', on_pose, 10)
node.create_subscription(Twist, '/turtle1/cmd_vel', lambda value: commands.append(value), 100)
def spin(seconds=0.1):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.02)
def wait(future, timeout=60):
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
    assert future.done(), 'Operation timed out'
    return future.result()
def call(name, kind, request):
    client = node.create_client(kind, name)
    assert client.wait_for_service(timeout_sec=10)
    result = wait(client.call_async(request))
    node.destroy_client(client)
    return result
pose_client = ActionClient(node, GoToPose, '/go_to_pose')
square_client = ActionClient(node, DrawSquare, '/draw_square')
assert pose_client.wait_for_server(timeout_sec=10)
assert square_client.wait_for_server(timeout_sec=10)
def limits(linear, angular):
    return call('/set_speed_limits', SetSpeedLimits,
                SetSpeedLimits.Request(max_linear_speed=linear, max_angular_speed=angular))
def reset():
    assert call('/reset_turtle', Trigger, Trigger.Request()).success
    spin()
    assert abs(pose.x-5.544445)<0.001 and abs(pose.y-5.544445)<0.001
    assert abs(pose.theta)<0.001

def goal(x,y,yaw):
    return GoToPose.Goal(target=TargetPose(x=float(x),y=float(y),yaw=float(yaw)))
def move(x,y,yaw):
    feedback=[]
    handle=wait(pose_client.send_goal_async(goal(x,y,yaw),feedback_callback=lambda f: feedback.append(f)))
    assert handle.accepted
    result=wait(handle.get_result_async())
    assert result.status==4 and result.result.success, result
    spin()
    distance=math.hypot(pose.x-x,pose.y-y)
    angle=abs(math.atan2(math.sin(pose.theta-yaw),math.cos(pose.theta-yaw)))
    assert distance<=0.1 and angle<=0.05,(distance,angle)
    assert feedback
    assert commands[-1].linear.x==0 and commands[-1].angular.z==0
    print(f'PASS pose {(x,y,yaw)} distance={distance:.4f} yaw={angle:.4f}',flush=True)

try:
    assert limits(2.0,2.0).success
    for linear,angular in [(0.0,2.0),(-1.0,2.0),(2.0,0.0),(2.0,-1.0),(float('nan'),2.0)]:
        assert not limits(linear,angular).success
    values=call('/go_to_pose_server/get_parameters',GetParameters,
                GetParameters.Request(names=['max_linear_speed','max_angular_speed']))
    assert [v.double_value for v in values.values]==[2.0,2.0]
    for name in ['max_linear_speed','max_angular_speed']:
        for value in [1.5,0.0,-1.0,2.0]:
            response=call('/go_to_pose_server/set_parameters',SetParameters,
                          SetParameters.Request(parameters=[Parameter(name,value=value).to_parameter_msg()]))
            assert response.results[0].successful==(value>0)
    print('PASS atomic service and native parameter validation',flush=True)
    reset()
    for target in [(7,5.544445,0.7),(4,5.544445,-0.4),(4,8,2.6),(4,3,-2.0),(7,3,math.pi)]:
        move(*target)
    for target in [(0,5,0),(11,5,0),(5,-1,0),(5,5,float('nan'))]:
        assert not wait(pose_client.send_goal_async(goal(*target))).accepted
    print('PASS invalid pose rejection',flush=True)
    reset(); reset(); reset()
    started=time.monotonic(); move(7.5,5.544445,0); fast=time.monotonic()-started
    reset(); assert limits(0.4,0.5).success
    commands.clear(); started=time.monotonic(); move(7.5,5.544445,0); slow=time.monotonic()-started
    assert slow>fast*1.4,(slow,fast)
    assert max(abs(c.linear.x) for c in commands)<=0.40001
    assert max(abs(c.angular.z) for c in commands)<=0.50001
    print(f'PASS lower limits: fast={fast:.2f}s slow={slow:.2f}s',flush=True)
    reset(); assert limits(2.0,2.0).success
    handle=wait(pose_client.send_goal_async(goal(9,5.544445,0)))
    pending=handle.get_result_async(); spin(0.3)
    assert limits(0.3,0.4).success
    spin(0.15); commands.clear(); spin(0.3)
    assert commands and max(abs(c.linear.x) for c in commands)<=0.30001
    assert not wait(pose_client.send_goal_async(goal(3,3,0))).accepted
    wait(handle.cancel_goal_async()); outcome=wait(pending)
    assert outcome.status==5
    spin(); assert commands[-1].linear.x==0 and commands[-1].angular.z==0
    print('PASS live speed update, concurrency rejection and pose cancellation',flush=True)
    assert limits(2.0,2.0).success
    for side in [0.0,-1.0,6.0,float('nan')]:
        assert not wait(square_client.send_goal_async(DrawSquare.Goal(side_length=side))).accepted
    for repetition in range(2):
        feedback=[]
        handle=wait(square_client.send_goal_async(DrawSquare.Goal(side_length=2.0),feedback_callback=lambda f: feedback.append(f.feedback)))
        assert handle.accepted
        outcome=wait(handle.get_result_async(),timeout=90)
        assert outcome.status==4 and outcome.result.sides_completed==4 and outcome.result.success,outcome
        assert set(f.current_side for f in feedback)=={1,2,3,4}
        spin(); assert math.hypot(pose.x-5.544445,pose.y-5.544445)<=0.1
        print(f'PASS square repetition {repetition+1}: four sides and feedback',flush=True)
    handle=wait(square_client.send_goal_async(DrawSquare.Goal(side_length=2.0)))
    pending=handle.get_result_async(); spin(0.7)
    wait(handle.cancel_goal_async()); outcome=wait(pending)
    assert outcome.status==5
    spin(); assert commands[-1].linear.x==0 and commands[-1].angular.z==0
    reset()
    print('PASS square cancellation and repeated reset',flush=True)
finally:
    node.destroy_node(); rclpy.shutdown()
