#!/usr/bin/env python3

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
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
import launch


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world_file_name = 'clearpath_playpen.world'
    #world_file_name='empty.world'
    world = os.path.join(get_package_share_directory('kmr_simulation'), 'worlds', world_file_name)

    launch_file_dir = os.path.join(get_package_share_directory('kmr_simulation'), 'launch')

    return LaunchDescription([
        launch.actions.ExecuteProcess(
                cmd=['gazebo', '--verbose', world, '-s', 'libgazebo_ros_init.so'],
                output='screen'),

        #Node(package='gazebo_ros', node_executable='spawn_entity.py',
        #                    arguments=['-entity', 'kmr', '-topic', '/robot_description'],
        #                    output='screen')

    ])
