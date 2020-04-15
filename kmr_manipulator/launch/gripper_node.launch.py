import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    return LaunchDescription([

        Node(
            package="kuka_manipulator",
            node_executable="gripper_node.py",
            node_name="gripper_node",
            output='screen',
            parameters=[],
            emulate_tty=True,
            ),

        ])