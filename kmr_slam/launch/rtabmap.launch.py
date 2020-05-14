from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')

    rtabmap_rviz_parameters=[{
          'frame_id':'base_footprint',
          'use_sim_time':use_sim_time,
          'subscribe_rgbd':True,
          'subscribe_rgb':False,
          'subscribe_scan':False,
          'subscribe_depth':False,
          'queue_size':20,
          'rgbd_cameras':1}]

    rtabmap_parameters=[{
        'frame_id':'base_footprint',
        'use_sim_time':use_sim_time,
        'subscribe_rgbd':True,
        'subscribe_rgb':False,
        'subscribe_scan':False,
        'subscribe_depth':False,
        'config_paht':'', #Path of a config files containing RTAB-Map's parameters. 
        'approx_sync':True,
        'queue_size':20,
        'rgbd_cameras':1,
    }]

    remappings1=[
          ('rgb/image', '/camera1/camera/color/image_raw'),
          ('rgb/camera_info', '/camera1/camera/color/camera_info'),
          #('rgbd_image','/rgbd_image0'),
          ('depth/image', '/camera1/camera/depth/image_rect_raw')]

    remappings2=[
        ('rgb/image', '/camera2/camera/color/image_raw'),
        ('rgb/camera_info', '/camera2/camera/color/camera_info'),
        ('rgbd_image','/rgbd_image1'),
        ('depth/image', '/camera2/camera/depth/image_rect_raw')

    ]

    remappings_gazebo=[
        ('rgb/image','mycamera/image_demo'),
        ('rgb/camera_info', 'mycamera/image_demo/camera_info'),
        ('depth/image', 'mycamera/depth_demo')]

    remappings_scan=[
	    ('scan','/scan_1')]

          

    return LaunchDescription([
        # Set env var to print messages to stdout immediately
        SetEnvironmentVariable('RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED', '1'),

        # Launch arguments
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use simulation (Gazebo) clock if true'),

        # Nodes to launch
        Node(
            package='rtabmap_ros', node_name='sync1', node_executable='rgbd_sync', output='screen',
            parameters=[{'approx_sync':True, 'use_sim_time':use_sim_time}],
            remappings=remappings_gazebo,
            emulate_tty=True),
        #Node(
        #    package='rtabmap_ros', node_name='sync2', node_executable='rgbd_sync', output='screen',
        #    parameters=[{'approx_sync':True, 'use_sim_time':use_sim_time}],
        #    remappings=remappings2,
        #    emulate_tty=True),

        Node(
            package='rtabmap_ros', node_executable='rtabmap', output='screen',
            parameters=rtabmap_parameters,
	        remappings=remappings1,
            #arguments=['-d'], #-d deletes the database before starting, otherwise the previous mapping session is loaded 
            emulate_tty=True,),

        Node(
            package='rtabmap_ros', node_executable='rtabmapviz', output='screen',
            parameters=rtabmap_parameters,
	        remappings=remappings1,
            emulate_tty=True),
    ])
