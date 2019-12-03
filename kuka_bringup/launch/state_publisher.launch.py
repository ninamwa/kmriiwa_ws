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

    urdf_file_name = 'kuka_correct.urdf'
    urdf = os.path.join(
        get_package_share_directory('kuka_bringup'),
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
