
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
    world = os.path.join(get_package_share_directory('kuka_simulation'), 'worlds', world_file_name)

    launch_file_dir = os.path.join(get_package_share_directory('kuka_simulation'), 'launch')

    return LaunchDescription([
        launch.actions.ExecuteProcess(
                cmd=['gazebo', '--verbose', world, '-s', 'libgazebo_ros_init.so'],
                output='screen'),

        #Node(package='gazebo_ros', node_executable='spawn_entity.py',
        #                    arguments=['-entity', 'kuka', '-topic', '/robot_description'],
        #                    output='screen')

    ])
