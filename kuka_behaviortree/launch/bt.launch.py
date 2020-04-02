import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    gripper_node_launch_file_dir = os.path.join(get_package_share_directory('kuka_manipulator'),'launch')
    moveit_launch_file_dir = os.path.join(get_package_share_directory('kuka_moveit2'), 'launch')

    plugin_lib_names =['close_gripper_action_bt_node', 'open_gripper_action_bt_node','move_manipulator_action_bt_node','plan_manipulator_path_action_bt_node','object_search_action_bt_node','frame_empty_condition_bt_node']
    #xml_file_name = 'manipulator_tree.xml'
    xml_file_name = 'test.xml'
    xml = os.path.join(
        get_package_share_directory('kuka_behaviortree'),
        'behavior_trees',
        xml_file_name)

    
    connection_type_TCP='TT'
    robot = 'KMR1'
    param_dir = LaunchConfiguration(
        'param_dir',
        default=os.path.join(
            get_package_share_directory('kuka_communication'),
            'param',
            'bringup.yaml'))

    return LaunchDescription([

        DeclareLaunchArgument(
            'bt_xml_filename', default_value='kuka_behaviortree/behavior_trees/manipulator_tree.xml',
            description='File to read BT tree from'),

        #DeclareLaunchArgument(
        #    'plugin_lib_names',
        #    default_value='',
        #    description='plugin-libs'),


        IncludeLaunchDescription(
               PythonLaunchDescriptionSource([gripper_node_launch_file_dir, '/gripper_node.launch.py']),
        ),

        #IncludeLaunchDescription(
        #    PythonLaunchDescriptionSource([moveit_launch_file_dir, '/moveit.launch.py']),
        #),

        #Node(
        #    package="kuka_communication",
        #    node_executable="lbr_commands_node.py",
        #    node_name="lbr_commands_node",
        #    output="screen",
        #    emulate_tty=True,
        #    arguments=['-c', connection_type_TCP, '-ro', robot],
        #    parameters=[param_dir]),


        Node(
            package="kuka_behaviortree",
            node_executable="behavior_tree_node2",
            node_name="behavior_tree_node2",
            output='screen',
            parameters=[{'bt_xml_filename': xml},{'plugin_lib_names': plugin_lib_names}],
            emulate_tty=True,
            ),

        ])