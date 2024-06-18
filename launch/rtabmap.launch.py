#!/usr/bin/env python3
#

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node



import xacro

def generate_launch_description():
    
    self_dir = get_package_share_directory('deliverybot')

    rviz = LaunchConfiguration('rviz')
    rtabmap_viz = LaunchConfiguration('rtabmap_viz')

    strategy = LaunchConfiguration('strategy')
    feature = LaunchConfiguration('feature')
    nn = LaunchConfiguration('nn')
    max_depth = LaunchConfiguration('max_depth')
    min_inliers = LaunchConfiguration('min_inliers')
    inlier_distance = LaunchConfiguration('inlier_distance')
    local_map = LaunchConfiguration('local_map')
    odom_info_data = LaunchConfiguration('odom_info_data')
    wait_for_transform = LaunchConfiguration('wait_for_transform')
    
    rviz_cmd = DeclareLaunchArgument('rviz', default_value='true')
    rtabmap_viz_cmd = DeclareLaunchArgument('rtabmap_viz', default_value='false') 
    strategy_cmd = DeclareLaunchArgument('strategy', default_value='"0"', 
                                         description='Strategy: 0=Frame-to-Map 1=Frame-to-Frame')
    feature_cmd = DeclareLaunchArgument('feature', default_value='"6"', 
                                        description='Feature type: 0=SURF 1=SIFT 2=ORB 3=FAST/FREAK 4=FAST/BRIEF 5=GFTT/FREAK 6=GFTT/BRIEF 7=BRISK')
    nn_cmd= DeclareLaunchArgument('nn', default_value='"3"',
                                  description='''Nearest neighbor strategy : 0=Linear, 1=FLANN_KDTREE, 2=FLANN_LSH, 3=BRUTEFORCE
        Set to 1 for float descriptor like SIFT/SURF
        Set to 3 for binary descriptor like ORB/FREAK/BRIEF/BRISK''')
    max_depth_cmd= DeclareLaunchArgument('max_depth', default_value='4.0',
                                         description='Maximum features depth (m)')
    min_inliers_cmd= DeclareLaunchArgument('min_inliers', default_value='"20"',
                                           description='Minimum visual correspondences to accept a transformation (m)')
    inlier_distance_cmd= DeclareLaunchArgument('inlier_distance', default_value='"0.02"',
                                               description='RANSAC maximum inliers distance (m)')
    local_map_cmd= DeclareLaunchArgument('local_map', default_value='"1000"',
                                         description='Local map size: number of unique features to keep track')
    odom_info_data_cmd= DeclareLaunchArgument('odom_info_data', default_value='"true"',
                                              description='Fill odometry info messages with inliers/outliers data.')
    wait_for_transform_cmd= DeclareLaunchArgument('wait_for_transform', default_value='0.1',
                                                  description='')


    camera_left_rgbd_cmd = Node(
            package='rtabmap_sync',
            executable='rgbd_sync',
            name='camera_left_rgbd_sync',
            output='screen',
            parameters=[{
                'approx_sync': True,
                'approx_sync_max_interval':0.04,
                'sync_queue_size': 10
            }],
            remappings=[
                ('/rgb/image', '/camera_left/image_raw'),
                ('/depth/image', '/camera_left/depth/image_raw'),
                ('/rgb/camera_info', '/camera_left/camera_info'),
                ('/rgbd_image', '/camera_left/rgbd_image')
            ]
        )
    camera_right_rgbd_cmd = Node(
            package='rtabmap_sync',
            executable='rgbd_sync',
            name='camera_right_rgbd_sync',
            output='screen',
            parameters=[{
                'approx_sync': True,
                'sync_queue_size': 10
            }],
            remappings=[
                ('/rgb/image', '/camera_right/image_raw'),
                ('/depth/image', '/camera_right/depth/image_raw'),
                ('/rgb/camera_info', '/camera_right/camera_info'),
                ('/rgbd_image', '/camera_right/rgbd_image')
            ]
        )

    # rtbamap RGBD odometry node
    rtabmap_odom_cmd = Node(
        name='rgbd_odometry',
        package='rtabmap_odom',
        executable='rgbd_odometry',
        output='screen',
        parameters=[
            {
                'subscribe_rgbd':True,
                'frame_id':'base_link',
                'rgbd_cameras':2,
                'wait_for_transform':wait_for_transform,
                'Odom/Strategy':strategy,
                'OdomF2M/BundleAdjustment':"0",
                'Vis/EstimationType':"0",
                'Vis/FeatureType':feature,
                'Vis/CorGuessWinSize':"0",
                'Vis/CorNNType':nn,
                'Vis/MaxDepth':"0",
                'Vis/MinInliers':min_inliers,
                'Vis/InlierDistance':inlier_distance,
                'OdomF2M/MaxSize':local_map,
                'Odom/FillInfoData':odom_info_data,
            }
        ],
        remappings=[
            ('/rgbd_image0', '/camera_left/rgbd_image'),
            ('/rgbd_image1', '/camera_right/rgbd_image')
        ]
     )
    
    # rtabmap-slam node for mapping and localization
    rtabmap_slam_cmd = Node(
        name='rtabmap_slam',
        package='rtabmap_slam',
        executable='rtabmap',
        output='screen',
        arguments=['--delete_db_on_start'],
        parameters=[
            {
                'subscribe_depth':False,
                'subscribe_rgb':False,
                'subscribe_rgbd':True,
                'rgbd_cameras':2,
                'frame_id':'base_link',
                'gen_scan':True,
                'wait_for_transform':wait_for_transform,
                'map_negative_poses_ignored':False,
                'map_negative_scan_empty_ray_tracing':False,
                'Grid/FromDepth':False,
                'Vis/EstimationType':"0",
                'Vis/MinInliers':"10",
                'Vis/InlierDistance':inlier_distance,
            }
        ],
        remappings=[
            ('/rgbd_image0', '/camera_left/rgbd_image'),
            ('/rgbd_image1', '/camera_right/rgbd_image')
        ]
    )

    # <!-- Visualisation RTAB-Map -->
    rtabmap_viz_node_cmd = Node(
        name='rtabmap_viz',
        package='rtabmap_rviz',
        executable='rtabmap_viz',
        output='screen',
        condition=IfCondition(rtabmap_viz),
        arguments=['-d', os.path.join(get_package_share_directory('rtabmap_demos'), 'launch', 'config', 'rgbd_gui.ini')],
        parameters=[
            {
                'subscribe_depth':False,
                'subscribe_rgbd':True,
                'rgbd_cameras':2,
                'frame_id':'base_link',
                'wait_for_transform':wait_for_transform,
                'subscribe_odom_info':odom_info_data,
            }
        ],
        remappings=[
            ('/rgbd_image0', '/camera_left/rgbd_image'),
            ('/rgbd_image1', '/camera_right/rgbd_image')
        ]
    )

    # Visualization RVIZ 
#   <node if="$(arg rviz)" pkg="rviz" exec="rviz" name="rviz" args="-d $(find rtabmap_demos)/launch/config/rgbd.rviz"/>
    rviz_node_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        condition=IfCondition(rviz),
        arguments=['-d', os.path.join(get_package_share_directory('rtabmap_demos'), 'launch', 'config' ,'rgbd.rviz')]
    )


    ld = LaunchDescription()

    # Add the commands to the launch description
    ld.add_action(rviz_cmd)
    ld.add_action(rtabmap_viz_cmd)
    ld.add_action(strategy_cmd)
    ld.add_action(feature_cmd)
    ld.add_action(nn_cmd)
    ld.add_action(max_depth_cmd)
    ld.add_action(min_inliers_cmd)
    ld.add_action(inlier_distance_cmd)
    ld.add_action(local_map_cmd)
    ld.add_action(odom_info_data_cmd)
    ld.add_action(wait_for_transform_cmd)

    ld.add_action(camera_left_rgbd_cmd)
    ld.add_action(camera_right_rgbd_cmd)
    ld.add_action(rtabmap_odom_cmd)
    ld.add_action(rtabmap_slam_cmd)
    ld.add_action(rtabmap_viz_node_cmd)
    ld.add_action(rviz_node_cmd)

    return ld
