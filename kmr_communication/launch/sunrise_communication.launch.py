# Copyright 2019 Nina Marie Wahl and Charlotte Heggem.
# Copyright 2019 Norwegian University of Science and Technology.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

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

    connection_type_TCP='TCP'
    connection_type_UDP ='UDP'

    robot="KMR2"

    param_dir = LaunchConfiguration(
        'param_dir',
        default=os.path.join(
            get_package_share_directory('kmr_communication'),
            'param',
            'bringup.yaml'))

    return LaunchDescription([
        DeclareLaunchArgument(
            'param_dir',
            default_value=param_dir,
            description='Full path to parameter file to load'),

        launch_ros.actions.Node(
            package="tf2_ros",
            node_executable="static_transform_publisher",
            output="screen",
            arguments=['0','0','0','0','0','0','laser_B4_link','scan_2'],
           ),

        launch_ros.actions.Node(
            package="tf2_ros",
            node_executable="static_transform_publisher",
            output="screen",
            arguments=['0','0','0','0','0','0','laser_B1_link','scan'],
           ),

        launch_ros.actions.Node(
            package="kmr_communication",
            node_executable="kmp_commands_node.py",
            node_name="kmp_commands_node",
            output="screen",
            emulate_tty=True,
            arguments=['-c', connection_type_TCP,'-ro', robot],
            parameters=[param_dir]),

        launch_ros.actions.Node(
           package="kmr_communication",
           node_executable="kmp_laserscan_node.py",
           node_name="kmp_laserscan_node",
           output="screen",
           emulate_tty=True,
           arguments=['-c', connection_type_TCP, '-ro', robot],
           parameters=[param_dir]),

        launch_ros.actions.Node(
           package="kmr_communication",
           node_executable="kmp_odometry_node.py",
           node_name="kmp_odometry_node",
           output="screen",
           emulate_tty=True,
           arguments=['-c', connection_type_TCP,'-ro',robot],
           parameters=[param_dir]),

        launch_ros.actions.Node(
           package="kmr_communication",
           node_executable="kmp_statusdata_node.py",
           node_name="kmp_statusdata_node",
           output="screen",
           emulate_tty=True,
           arguments=['-c', connection_type_TCP, '-ro', robot],
           parameters=[param_dir]),

        launch_ros.actions.Node(
            package="kmr_communication",
            node_executable="lbr_commands_node.py",
            node_name="lbr_commands_node",
            output="screen",
            emulate_tty=True,
            arguments=['-c', connection_type_TCP, '-ro', robot],
            parameters=[param_dir]),

        launch_ros.actions.Node(
            package="kmr_communication",
            node_executable="lbr_statusdata_node.py",
            node_name="lbr_statusdata_node",
            output="screen",
            emulate_tty=True,
            arguments=['-c', connection_type_TCP, '-ro', robot],
            parameters=[param_dir]),

        launch_ros.actions.Node(
            package="kmr_communication",
            node_executable="lbr_sensordata_node.py",
            node_name="lbr_sensordata_node",
            output="screen",
            emulate_tty=True,
            arguments=['-c', connection_type_TCP, '-ro', robot],
            parameters=[param_dir]),
    ])
