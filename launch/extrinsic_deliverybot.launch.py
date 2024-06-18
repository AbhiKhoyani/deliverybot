#!/usr/bin/env python3
#
# Copyright 2019 ROBOTIS CO., LTD.
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
#
# Authors: Joep Tool

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import xacro

def generate_launch_description():
    self_dir = get_package_share_directory('deliverybot')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')
    z_pose = LaunchConfiguration('z_pose', default='0.0')

    world = os.path.join(
        self_dir,
        'worlds',
        'box.world'
    )

    xacro_path = os.path.join(
        self_dir,
        'urdf',
        'deliverybot.urdf.xacro'
    )

    sdf_path = os.path.join(
        self_dir,
        'models',
        'model.sdf'
    )

    robot_desc_config = xacro.process_file(xacro_path)
    robot_desc = robot_desc_config.toxml()

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world}.items()
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )

    urdf_spawner_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
        respawn=False,
        arguments=["-file", sdf_path, "-entity","deliveryBot", '-x', x_pose,
                   '-y', y_pose, '-z', z_pose]
    )

    rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(self_dir, 'rviz', 'rviz_config.rviz')]
    )

    robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_desc
            }],
        )
    
    # multi lidar calibration launch
    mlc_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('multi_lidar_calibration'), 'launch', 'multi_lidar_calibration_ndt.launch.xml')
        ),
        launch_arguments={'initial_pose': "[0.03, 0.0, 0.03, 0.0, -1.46, 0.0]",
                          'input/source_pointcloud':'/camera_left/points',
                          'input/target_pointcloud':'/camera_right/points'}.items()
    )


    ld = LaunchDescription()

    # Add the commands to the launch description
    # ld.add_action(gzserver_cmd)
    # ld.add_action(gzclient_cmd)
    # ld.add_action(robot_state_publisher_cmd)
    # ld.add_action(rviz_cmd)
    # ld.add_action(urdf_spawner_cmd)
    ld.add_action(mlc_cmd)

    return ld
