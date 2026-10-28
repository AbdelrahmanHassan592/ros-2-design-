"""Start the lab and automatically request one square."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description() -> LaunchDescription:
    """Include lab bringup, then invoke the ROS action CLI."""
    launch = Path(get_package_share_directory('turtle_lab_bringup')) / 'launch/lab1.launch.py'
    return LaunchDescription([
        DeclareLaunchArgument('side_length', default_value='2.0'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(launch))),
        TimerAction(period=3.0, actions=[ExecuteProcess(cmd=[
            'ros2', 'action', 'send_goal', '--feedback', '/draw_square',
            'turtle_lab_interfaces/action/DrawSquare',
            ['{side_length: ', LaunchConfiguration('side_length'), '}']], output='screen')])])
