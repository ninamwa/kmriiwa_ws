import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return file.read()
    except EnvironmentError: # parent of IOError, OSError *and* WindowsError where available
        return None

def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.load(file)
    except EnvironmentError: # parent of IOError, OSError *and* WindowsError where available
        return None


def generate_launch_description():
    # moveit_cpp.yaml is passed by filename for now since it's node specific
    moveit_cpp_yaml_file_name = get_package_share_directory('kmr_moveit2') + "/config/moveit_cpp.yaml"

    # Component yaml files are grouped in separate namespaces
    robot_description_config = load_file('kmr_bringup', 'urdf/kmriiwa.urdf')
    robot_description = {'robot_description' : robot_description_config}

    robot_description_semantic_config = load_file('kmr_moveit2', 'config/iiwa14.srdf')
    robot_description_semantic = {'robot_description_semantic' : robot_description_semantic_config}

    kinematics_yaml = load_yaml('kmr_moveit2', 'config/kinematics.yaml')
    robot_description_kinematics = {'robot_description_kinematics' : kinematics_yaml }

    controllers_yaml = load_yaml('kmr_moveit2', 'config/controllers.yaml')
    moveit_controllers = {'moveit_simple_controller_manager' : controllers_yaml }

    ompl_planning_pipeline_config = { 'ompl' : {
        'planning_plugin' : 'ompl_interface/OMPLPlanner',
        'request_adapters' : """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""" ,
        'start_state_max_bounds_error' : 0.1 } }
    ompl_planning_yaml = load_yaml('kmr_moveit2', 'config/ompl_planning.yaml')
    ompl_planning_pipeline_config['ompl'].update(ompl_planning_yaml)

    planner_plugin={'planner_plugin' : 'ompl_interface/OMPLPlanner'}

    state_publisher_launch_file_dir = os.path.join(get_package_share_directory('kmr_bringup'), 'launch')

    # MoveItCpp demo executable
    run_moveit_node = Node(node_name='run_moveit',
                               package='kmr_moveit2',
                               node_executable='run_moveit',
                               output='screen',
                               emulate_tty=True,
                               parameters=[moveit_cpp_yaml_file_name,
                                           robot_description,
                                           robot_description_semantic,
                                           kinematics_yaml,
                                           ompl_planning_pipeline_config,
                                           moveit_controllers])
                                           
    # RViz
    rviz_config_file = get_package_share_directory('kmr_moveit2') + "/rviz/moveit.rviz"
    rviz_node = Node(package='rviz2',
                     node_executable='rviz2',
                     node_name='rviz2',
                     output='screen',
                     arguments=['-d', rviz_config_file],
                     parameters=[robot_description])

    # Publish base link TF
    static_tf = Node(package='tf2_ros',
                     node_executable='static_transform_publisher',
                     node_name='static_transform_publisher',
                     output='screen',
                     arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'world', 'base_footprint'])

    # Fake joint driver
    fake_joint_driver_node = Node(package='fake_joint_driver',
                                  node_executable='fake_joint_driver_node',
                                  parameters=[os.path.join(get_package_share_directory("kmr_moveit2"), "config", "iiwa_controllers.yaml"),
                                              os.path.join(get_package_share_directory("kmr_moveit2"), "config", "start_positions.yaml"),
                                              robot_description],
                                  output='screen',
                                  )

    return LaunchDescription([ static_tf,rviz_node,run_moveit_node,
     IncludeLaunchDescription(
           PythonLaunchDescriptionSource([state_publisher_launch_file_dir, '/state_publisher.launch.py']),
       ),
        ])