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
import launch_ros.actions
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    resolution = LaunchConfiguration('resolution', default='0.05')
    publish_period_sec = LaunchConfiguration('publish_period_sec', default='1.0')

    rviz_config_dir = os.path.join(
        get_package_share_directory('kuka_cartographer'),
        'rviz',
        'kuka_cartographer.rviz'),

    rviz_config_dir2 = os.path.join(get_package_share_directory('realsense_examples'), 'config', 'rs_cartographer.rviz')
    rviz_config_dir3 = os.path.join(get_package_share_directory('kuka_cartographer'),'rviz','kuka_cartographer_withd435.rviz'),

    cartographer_config_dir = os.path.join(
        get_package_share_directory('kuka_cartographer'),
        'config')

    configuration_basename = LaunchConfiguration('configuration_basename', default='kuka_2d.lua'),

    state_pub_pkg_dir = LaunchConfiguration(
        'state_pub_pkg_dir',
        default=os.path.join(get_package_share_directory('kuka_bringup'), 'launch')),

    state_publisher_launch_file_dir = os.path.join(get_package_share_directory('kuka_bringup'), 'launch')

    return LaunchDescription([

        DeclareLaunchArgument(
            'resolution',
            default_value=resolution,
            description='Resolution of a grid cell in the published occupancy grid'),

        DeclareLaunchArgument(
            'publish_period_sec',
            default_value=publish_period_sec,
            description='OccupancyGrid publishing period'),

        #    launch_ros.actions.Node(
        #        package='cartographer_ros',
        #        node_executable='cartographer_node',
        #        node_name='cartographer_node',
        #        parameters=[{'use_sim_time': use_sim_time}],
        #       arguments=['-configuration_directory', cartographer_config_dir, 'configuration_basename',configuration_basename],
        #       output='screen'),

        # launch_ros.actions.Node(
        #        package='cartographer_ros',
        #        node_executable='occupancy_grid_node',
        #        node_name='occupancy_grid_node',
        #        output='screen',
        #        parameters=[{'use_sim_time': use_sim_time}],
        #        arguments=['-resolution', resolution, '-publish_period_sec', publish_period_sec]),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([state_publisher_launch_file_dir, '/state_publisher.launch.py']),
        ),

        launch_ros.actions.Node(
            package='rviz2',
            node_executable='rviz2',
            node_name='rviz2',
            arguments=['-d', rviz_config_dir3],
 	        parameters=[{'use_sim_time': use_sim_time}],
            output='screen'),
    ])
