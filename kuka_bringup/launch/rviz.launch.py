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

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    rviz_config_dir = os.path.join(
            get_package_share_directory('kuka_bringup'),
            'rviz',
            'tf3.rviz')

    urdf_file_name = 'kuka_withD435.urdf'
    urdf = os.path.join(
        get_package_share_directory('kuka_bringup'),
        'urdf',
        urdf_file_name)

    map_name='playpen_map.yaml'
    map_path=os.path.join(get_package_share_directory('kuka_navigation2'),
        'map',
        map_name)



    return LaunchDescription([



        launch_ros.actions.Node(
            package='rviz2',
            node_executable='rviz2',
            node_name='rviz2',
            arguments=['-d', rviz_config_dir],
            output='screen'),


        launch_ros.actions.Node(package='kuka_bringup', node_executable='dummy_joint_states', output='screen'),
        #launch_ros.actions.Node(
        #    package='joint_state_publisher',
        #    node_executable='joint_state_publisher',
        #    output='screen',
        #    arguments=[urdf],
        #    parameters=[{'use_gui': True}]),


    ])
