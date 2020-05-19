# Copyright (c) 2019 Intel Corporation
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

# /* Author: Gary Liu */

import os
import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    # config the serial number and base frame id of each camera
    camera1_base_frame_id = LaunchConfiguration('base_frame_id', default='d435_base_front_link')
    camera1_serial_no = LaunchConfiguration('serial_no', default="'12'")
    camera1_optical_frame_id = LaunchConfiguration('optical_frame_id', default='camera1_color_optical_frame_link')

    camera2_base_frame_id = LaunchConfiguration('base_frame_id', default='d435_base_right_link')
    camera2_serial_no = LaunchConfiguration('serial_no', default="'011422070886'")
    camera2_optical_frame_id = LaunchConfiguration('optical_frame_id', default='camera2_color_optical_frame_link')


    camera3_base_frame_id = LaunchConfiguration('base_frame_id', default='d435_base_left_link')
    camera3_serial_no = LaunchConfiguration('serial_no', default="'831612071154'")
    camera3_optical_frame_id = LaunchConfiguration('optical_frame_id', default='camera3_color_optical_frame_link')


    camera1_node = Node(
        package='realsense_node',
        node_executable='realsense_node',
        node_namespace="/camera1",
        output='screen',
        emulate_tty=True,
        remappings=[('/camera1/camera/pointcloud','/points2_1')],
        parameters=[{'serial_no':"'011422072245'",
		                'optical_frame_id': camera1_optical_frame_id,
                        'base_frame_id': camera1_base_frame_id,
                        'enable_pointcloud': True,
                        'dense_pointcloud': True,
                        'color0.enabled': True,
                        'depth0.enabled': True,
                        'infra1.enabled': False,
                        'infra2.enabled': False}]

        )
    camera2_node = Node(
        package='realsense_node',
        node_executable='realsense_node',
        node_namespace="/camera2",
        output='screen',
	    emulate_tty=True,
        remappings=[('/camera2/camera/pointcloud','/points2_2')],
        parameters=[{'serial_no': "'011422070886'", 
  	 	             'optical_frame_id': camera2_optical_frame_id,
                     'base_frame_id': camera2_base_frame_id,
                     'enable_pointcloud': True,
                     'dense_pointcloud': True,
                     'color0.enabled': True,
                     'depth0.enabled': True,
                     'infra1.enabled': False,
		             'infra2.enabled': False}]
        )
    camera3_node = Node(
        package='realsense_node',
        node_executable='realsense_node',
        node_namespace="/camera3",
        output='screen',
        emulate_tty=True,
        remappings=[('/camera3/camera/pointcloud','/points2_3')],
        parameters=[{'serial_no':camera3_serial_no,
                     'optical_frame_id': camera3_optical_frame_id,
                     'base_frame_id': camera3_base_frame_id,
                     'enable_pointcloud': True,
                     'dense_pointcloud': True,
                     'color0.enabled': True,
                     'depth0.enabled': True,
                     'infra1.enabled': False,
		             'infra2.enabled': False}]
        )
    #camera1_node
    return launch.LaunchDescription([camera1_node,camera2_node,camera3_node])
