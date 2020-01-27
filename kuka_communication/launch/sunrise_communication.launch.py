import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import launch_ros.actions
import sys
from rclpy.utilities import remove_ros_args
import argparse


def generate_launch_description(argv=sys.argv[1:]):

    connection_type='TCP'
    robot = argv[len(argv)-1]
    ip='12345'
    #connection = LaunchConfiguration('connection', default='UDP')

    return LaunchDescription([
        # DeclareLaunchArgument(
        #    'connection',
        #    default_value='UDP',
        #    description='What type of connection to use to KMR'),

        # launch_ros.actions.Node(
        #    package="kuka_communication",
        #    node_executable="test.py",
        #    node_name="test",
        #    output="screen",
        #    parameters=[{'connection':connection}])

        launch_ros.actions.Node(
            package="kuka_communication",
            node_executable="test.py",
            node_name="test",
            output="screen",
            arguments=['-c', connection_type, '-ro', robot]),





        launch_ros.actions.Node(
            package="kuka_communication",
            node_executable="kmp_commands_node.py",
            node_name="kmp_commands_node",
            output="screen",
            arguments=['-c', connection_type, '-ro', robot]),

        launch_ros.actions.Node(
            package="kuka_communication",
            node_executable="kmp_laserscan_node.py",
            node_name="kmp_laserscan_node",
            output="screen",
            arguments=['-c', connection_type, '-ro', robot]),

        launch_ros.actions.Node(
            package="kuka_communication",
            node_executable="kmp_odometry_node.py",
            node_name="kmp_odometry_node",
            output="screen",
            arguments=['-c', connection_type,'-ro',robot]),

        launch_ros.actions.Node(
            package="kuka_communication",
            node_executable="kmp_statusdata_node.py",
            node_name="kmp_statusdata_node",
            output="screen",
            arguments=['-c', connection_type, '-ro', robot]),



    ])
