"""Start the entire lab; YAML defaults can be overridden at launch."""
from pathlib import Path
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    """Build the four-node lab."""
    config = Path(get_package_share_directory('turtle_lab_bringup')) / 'config/params.yaml'
    defaults = yaml.safe_load(config.read_text())['go_to_pose_server']['ros__parameters']
    arguments = [DeclareLaunchArgument(name, default_value=str(value))
                 for name, value in defaults.items()]
    overrides = {name: ParameterValue(LaunchConfiguration(name), value_type=float)
                 for name in defaults}
    return LaunchDescription(arguments + [
        Node(package='turtlesim', executable='turtlesim_node', output='screen'),
        Node(package='turtle_lab_control', executable='go_to_pose_server',
             parameters=[str(config), overrides], output='screen'),
        Node(package='turtle_lab_control', executable='reset_service', output='screen'),
        Node(package='turtle_lab_control', executable='square_action_server', output='screen')])
