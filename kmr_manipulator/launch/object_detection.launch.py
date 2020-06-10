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
import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    object_param = os.path.join(get_package_share_directory('kmr_manipulator'), 'param', 'object_detection.yaml')
    object_model = os.path.join(get_package_share_directory('kmr_manipulator'), 'model', 'circbox.xml')

    camera_base_frame_id = LaunchConfiguration('base_frame_id', default='d435_manipulator_link')

    camera_serial_no = LaunchConfiguration('serial_no', default='831612071154')

    default_rviz = os.path.join(get_package_share_directory('object_analytics_node'), 'launch', 'rviz/default.rviz')

    # Realsense Node
    realsensenode = Node(
    package='realsense_node',
    node_executable='realsense_node',
    node_name='realsense_node',
    node_namespace="/ManipulatorCamera",
    output='screen',
    remappings=[('camera_depth_optical_frame', 'heii')],
    parameters=[{'serial_no':camera_serial_no,
		 'base_frame_id': camera_base_frame_id,
         'color0.enabled': True,
		 'depth0.enabled': True,
		 'align_depth': True,
		 'enable_pointcloud': True,
		 'infra1.enabled': False,
		 'infra2.enabled': False,}])

    # OpenVINO Node
    openvinonode = Node(
    package='dynamic_vino_sample', 
    node_executable='pipeline_with_params',
    arguments=['-config', object_param],
    remappings=[('/camera/color/image_raw', '/ManipulatorCamera/camera/color/image_raw'),
                ('/openvino_toolkit/object/detected_objects','/ros2_openvino_toolkit/detected_objects'),
                ('/openvino_toolkit/object/images', '/ros2_openvino_toolkit/image_rviz')],
    output='screen')

    # Object Analytics Node
    oanode = Node(
    package='object_analytics_node', node_executable='object_analytics_node', node_name='object_analytics_node',
    arguments=['--localization'],
    remappings=[('/object_analytics/detected_objects', '/ros2_openvino_toolkit/detected_objects'),
		('/object_analytics/rgb', '/ManipulatorCamera/camera/color/image_raw'),
		('/object_analytics/pointcloud', '/ManipulatorCamera/camera/pointcloud')],
    output='screen')

    # Object Search Node
    searchnode = Node(
    package='kmr_manipulator', node_executable='object_detection_node.py', node_name='object_detection_node',
    output='screen',
    emulate_tty=True)

    # object_analytics_rviz, add to launch description if desired
    oarviz = Node(
    package='object_analytics_rviz', node_executable='image_publisher',
    remappings=[('/object_analytics/rgb', 'ManipulatorCamera/camera/color/image_raw')],
    output='screen')

    oarviz2 = Node(
    package='object_analytics_rviz', node_executable='marker_publisher', 
    output='screen')

    rviz= Node(
    package='rviz2', node_executable='rviz2', output='screen',
    arguments=['--display-config', default_rviz])

    return launch.LaunchDescription([realsensenode,openvinonode,oanode,searchnode,oarviz,oarviz2,rviz])
