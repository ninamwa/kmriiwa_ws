from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')


    rtabmap_parameters=[{
        'frame_id':'base_footprint',
        'use_sim_time':use_sim_time,
        'subscribe_rgbd':False,
        'subscribe_rgb':False,
        'subscribe_scan':False,
        'subscribe_depth':False,
        'subscribe_scan_cloud':True,
        'config_paht':'', #Path of a config files containing RTAB-Map's parameters. 
        'approx_sync':True,
        'queue_size':20,
        'rgbd_cameras':3,

        'Grid/FromDepth':'False',
        'Grid/RayTracing':'True',
        'Grid/3D':'True',

        'Grid/NoiseFilteringRadius':'0',
        'Grid/NoiseFilteringMinNeighbors':'6',

        'Grid/RangeMax':'3.5',
        'Vis/MaxDepth':'5.0',
        'Grid/MaxObstacleHeight':'2.0',


        'Grid/NormalSegmentation':'False',
        
        'Reg/Strategy':'0',
        'Reg/Force3DoF':'True',
        'Vis/EstimationType':'0',
        'Vis/MinInliers':'10',
        'Vis/InlierDistance':'0.02',
        'Vis/CorGuessWinSize':'0',
        'OdomF2M/BundleAdjustment':'0',
        'Optimizer/Slam2D':'True',

    }]

    remappings1=[
          ('rgb/image', '/camera1/camera/color/image_raw'),
          ('rgb/camera_info', '/camera1/camera/color/camera_info'),
          ('rgbd_image','/rgbd_image0'),
          ('depth/image', '/camera1/camera/depth/image_rect_raw')]

    remappings2=[
        ('rgb/image', '/camera2/camera/color/image_raw'),
        ('rgb/camera_info', '/camera2/camera/color/camera_info'),
        ('rgbd_image','/rgbd_image1'),
        ('depth/image', '/camera2/camera/depth/image_rect_raw')
    ]

    remappings3=[
        ('rgb/image', '/camera3/camera/color/image_raw'),
        ('rgb/camera_info', '/camera3/camera/color/camera_info'),
        ('rgbd_image','/rgbd_image2'),
        ('depth/image', '/camera3/camera/depth/image_rect_raw')

    ]

    remappings_gazebo=[
        ('rgb/image','mycamera/image_demo'),
        ('rgb/camera_info', 'mycamera/image_demo/camera_info'),
        ('depth/image', 'mycamera/depth_demo')]

    remappings_scan=[
	    ('scan','/scan_1')]

    remappings_scancloud=[
        ('scan_cloud','/combined_cloud')]

          

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
            parameters=[{'approx_sync':False, 'use_sim_time':use_sim_time}],
            remappings=remappings1,
            emulate_tty=True),
        Node(
           package='rtabmap_ros', node_name='sync2', node_executable='rgbd_sync', output='screen',
           parameters=[{'approx_sync':False, 'use_sim_time':use_sim_time}],
           remappings=remappings2,
           emulate_tty=True),
        Node(
           package='rtabmap_ros', node_name='sync3', node_executable='rgbd_sync', output='screen',
           parameters=[{'approx_sync':False, 'use_sim_time':use_sim_time}],
           remappings=remappings3,
           emulate_tty=True),

        Node(
            package='rtabmap_ros', node_executable='rtabmap', output='screen',
            parameters=rtabmap_parameters,
	        remappings=remappings_scancloud,
            arguments=['-d'], #-d deletes the database before starting, otherwise the previous mapping session is loaded 
            emulate_tty=True,),

        Node(
            package='rtabmap_ros', node_executable='rtabmapviz', output='screen',
            parameters=rtabmap_parameters,
	        remappings=remappings_scancloud,
            emulate_tty=True),

        Node(
           package='pointcloud_to_laserscan',
           node_executable='laserscan_to_pointcloud_node',
           node_name='laserscan_to_pointcloud1',
           output='screen',
           parameters=[{'use_sim_time': use_sim_time}],
           remappings=[('scan_in', 'scan_1'),
                       ('cloud', 'cloud1')],
           ),
        Node(
           package='pointcloud_to_laserscan',
           node_executable='laserscan_to_pointcloud_node',
           node_name='laserscan_to_pointcloud2',
           output='screen',
           parameters=[{'use_sim_time': use_sim_time}],
           remappings=[('scan_in', 'scan_2'),
                       ('cloud', 'cloud2')],
           ),

         Node(
             package='rtabmap_ros', node_executable='point_cloud_aggregator', output='screen',
             #remappings=[('cloud1','points2_1'),('cloud2','points2_2'),('cloud3','points2_3')],
             parameters=[{'count': 2},{'approx_sync':True}],
             emulate_tty=True,),
    ])
