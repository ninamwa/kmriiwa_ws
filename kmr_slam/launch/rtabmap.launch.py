from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')
    icp_param=[{
            'approx_sync':True,
            'frame_id':'base_footprint',
            'queue_size':20,

            'Icp/RangeMax':'15.0',
            'Icp/PointToPlane':'True',
            'scan_normal_k':5,
            'scan_normal_radius':0.0,
            'publish_null_when_lost':False,

            'OdomF2M/BundleAdjustment':'0',
            'OdomF2M/MaxSize':'1000',
        }]
    odom_param=[{
        'Vis/EstimationType':'0',
        'subscribe_rgbd':True,
        'rgbd_cameras':3,
        'approx_sync':True,
        'frame_id':'base_footprint',
        'queue_size':20,

        'Vis/MaxDepth':'3.5',
        'Vis/EstimationType':'0',
        'Vis/CorGuessWinSize':'0',
        'Vis/MinInliers':'10',
        'Vis/InlierDistance':'0.02',
        'Vis/CorGuessWinSize':'0',
        'Vis/CorNNType':'3',
        'OdomF2M/BundleAdjustment':'0',
        'OdomF2M/MaxSize':'1000',
    }]


    rtabmap_parameters=[{
        'frame_id':'base_footprint',
        'use_sim_time':use_sim_time,
        'subscribe_rgbd':True,
        'subscribe_rgb':False,
        'subscribe_scan':False,
        'subscribe_depth':False,
        'subscribe_scan_cloud':False,
        'approx_sync':True,
        'queue_size':20,
        'rgbd_cameras':3,

        'Grid/FromDepth':'True',
        'Grid/RayTracing':'True',
        'Grid/3D':'False',

        'Grid/NoiseFilteringRadius':'0',
        'Grid/NoiseFilteringMinNeighbors':'6',

        'Grid/RangeMax':'3.5',
        #'Grid/RangeMax':'15.0',
        'Vis/MaxDepth':'5.0',
        'Grid/MaxObstacleHeight':'2.0',

        'Kp/MaxFeatures':'-1.0',

        #'Kp/MaxFeatures':'-1.0'

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
            parameters=[{'approx_sync':True, 'use_sim_time':use_sim_time}],
            remappings=remappings1,
            emulate_tty=True),
        Node(
           package='rtabmap_ros', node_name='sync2', node_executable='rgbd_sync', output='screen',
           parameters=[{'approx_sync':True, 'use_sim_time':use_sim_time}],
           remappings=remappings2,
           emulate_tty=True),
        Node(
           package='rtabmap_ros', node_name='sync3', node_executable='rgbd_sync', output='screen',
           parameters=[{'approx_sync':True, 'use_sim_time':use_sim_time}],
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

        # Node(
        #    package='pointcloud_to_laserscan',
        #    node_executable='laserscan_to_pointcloud_node',
        #    node_name='laserscan_to_pointcloud1',
        #    output='screen',
        #    parameters=[{'use_sim_time': use_sim_time}],
        #    remappings=[('scan_in', 'scan'),
        #                ('cloud', 'cloud1')],
        #    ),
        # Node(
        #    package='pointcloud_to_laserscan',
        #    node_executable='laserscan_to_pointcloud_node',
        #    node_name='laserscan_to_pointcloud2',
        #    output='screen',
        #    parameters=[{'use_sim_time': use_sim_time}],
        #    remappings=[('scan_in', 'scan_2'),
        #                ('cloud', 'cloud2')],
        #    ),

        #  Node(
        #      package='rtabmap_ros', node_executable='point_cloud_aggregator', output='screen',
        #      #remappings=[('cloud1','points2_1'),('cloud2','points2_2'),('cloud3','points2_3')],
        #      parameters=[{'count': 2},{'approx_sync':True}],
        #      emulate_tty=True,),

         Node(
             package='rtabmap_ros', node_executable='point_cloud_aggregator', output='screen',
             #remappings=[('cloud1','points2_1'),('cloud2','points2_2'),('cloud3','points2_3')],
             parameters=[{'count': 2},{'approx_sync':True}],
             emulate_tty=True,),

        # Node(
        #      package='rtabmap_ros', node_executable='icp_odometry', output='screen',
        #      parameters=icp_param,
        #      remappings=remappings_scancloud,
        #      emulate_tty=True),


        # Node(
        #    package='depthimage_to_laserscan',
        #    node_executable='depthimage_to_laserscan_node',
        #    node_name='depthimage_to_laserscan2',
        #    output='screen',
        #    parameters=[{'output_frame':'d435_base_right_link'},{'use_sim_time': use_sim_time},{'range_max':4.0},{'scan_height':150}],
        #    remappings=[('depth','/camera2/camera/depth/image_rect_raw'),
        #                ('depth_camera_info', '/camera2/camera/depth/camera_info'),
        #                ('scan', 'scan_4')],
        #    ),
        # Node(
        #     package='depthimage_to_laserscan',
        #     node_executable='depthimage_to_laserscan_node',
        #     node_name='depthimage_to_laserscan2',
        #     output='screen',
        #     parameters=[{'output_frame': 'd435_base_left_link'}, {'use_sim_time': use_sim_time}, {'range_max': 4.0},
        #                 {'scan_height': 150}],
        #     remappings=[('depth', '/camera3/camera/depth/image_rect_raw'),
        #                 ('depth_camera_info', '/camera3/camera/depth/camera_info'),
        #                 ('scan', 'scan_5')],
        #   ),

        #   Node(
        #    package='pointcloud_to_laserscan',
        #    node_executable='laserscan_to_pointcloud_node',
        #    node_name='laserscan_to_pointcloud3',
        #    output='screen',
        #    parameters=[{'use_sim_time': use_sim_time}],
        #    remappings=[('scan_in', 'scan_4'),
        #                ('cloud', 'cloud3')],
        #    ),
        # Node(
        #    package='pointcloud_to_laserscan',
        #    node_executable='laserscan_to_pointcloud_node',
        #    node_name='laserscan_to_pointcloud4',
        #    output='screen',
        #    parameters=[{'use_sim_time': use_sim_time}],
        #    remappings=[('scan_in', 'scan_5'),
        #                ('cloud', 'cloud4')],
        #    ),
    ])
