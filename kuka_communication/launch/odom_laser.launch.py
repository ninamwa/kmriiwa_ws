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
    #parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    #parser.add_argument('-ro', '--robot')
    #print(argv)
    #args = parser.parse_args(remove_ros_args(args=argv))
    #print(args)


    connection_type_TCP='TCP'
    connection_type_UDP = 'UDP'
    #robot = argv[len(argv)-1]
    #robot = args.robot
    robot='KMR2'


    return LaunchDescription([

        launch_ros.actions.Node(
            package="kuka_communication",
            node_executable="kmp_laserscan_node.py",
            node_name="kmp_laserscan_node",
            output="screen",
            emulate_tty=True,
            arguments=['-c', connection_type_UDP, '-ro', robot]),

        launch_ros.actions.Node(
            package="kuka_communication",
            node_executable="kmp_odometry_node.py",
            node_name="kmp_odometry_node",
            output="screen",
            emulate_tty=True,
            arguments=['-c', connection_type_UDP,'-ro',robot]),
    ])
