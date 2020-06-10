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
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    
    moveit_launch_file_dir = os.path.join(get_package_share_directory('kmr_moveit2'), 'launch')
    navigation_launch_file_dir = os.path.join(get_package_share_directory('kmr_navigation2'), 'launch')
    
    xml_file_name = 'test.xml'
    xml = os.path.join(
        get_package_share_directory('kmr_behaviortree'),
        'behavior_trees',
        xml_file_name)
    bt_param_dir = LaunchConfiguration(
        'bt_param_dir',
        default=os.path.join(
            get_package_share_directory('kmr_behaviortree'),
            'param',
            'param.yaml'))

    
    connection_type_TCP='TCP'
    robot = 'KMR2'
    param_dir = LaunchConfiguration(
        'param_dir',
        default=os.path.join(
            get_package_share_directory('kmr_communication'),
            'param',
            'bringup.yaml'))

    return LaunchDescription([

        Node(
            package="kmr_behaviortree",
            node_executable="behavior_tree_node",
            node_name="behavior_tree_node",
            output='screen',
            parameters=[{'bt_xml_filename': xml}, bt_param_dir],
            emulate_tty=True,
            ),

        ])