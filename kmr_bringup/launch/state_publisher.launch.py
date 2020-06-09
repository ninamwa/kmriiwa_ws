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
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    urdf_file_name = 'kmriiwa.urdf'
    urdf = os.path.join(
        get_package_share_directory('kmr_bringup'),
        'urdf',
        urdf_file_name)


    return LaunchDescription([

        launch_ros.actions.Node(
            package="robot_state_publisher",
            node_executable="robot_state_publisher",
            node_name="robot_state_publisher",
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=[urdf]),

    ])
