
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
            'model.rviz')

    urdf_file_name = 'kukatest.urdf'
    urdf = os.path.join(
        get_package_share_directory('kuka_bringup'),
        'urdf',
        urdf_file_name)


    return LaunchDescription([

        launch_ros.actions.Node(
            package='rviz2',
            node_executable='rviz2',
            node_name='rviz2',
            arguments=['-d', rviz_config_dir],
            output='screen'),


        #launch_ros.actions.Node(package='kuka_bringup', node_executable='dummy_joint_states', output='screen'),
        launch_ros.actions.Node(
            package='joint_state_publisher',
            node_executable='joint_state_publisher',
            output='screen',
            arguments=[urdf],
            parameters=[{'use_gui': True}]),


    ])
