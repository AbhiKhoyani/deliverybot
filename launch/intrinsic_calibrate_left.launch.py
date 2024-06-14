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

def generate_launch_description():
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    
    world = os.path.join(
        get_package_share_directory('deliverybot'),
        'worlds',
        # 'turtlebot3_empty.world'
        'empty.world'
    )

    robot_sdf_path = os.path.join(
        get_package_share_directory('deliverybot'),
        'models',
        'model.sdf'
    )
    
    checkerboard_sdf_path = os.path.join(
        get_package_share_directory('deliverybot'),
        'models','checkerboard',
        'checkerboard_8x6_003.sdf'
    )

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
        name='deliveryBot_spawner',
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
        respawn=False,
        arguments=["-file", robot_sdf_path, "-entity","deliveryBot", '-Y', '0.733']
    )
    
    checkerboard_left_spawner_cmd = Node(
        name='checkerboard_left_spawner',
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
        respawn=False,
        arguments=["-file", checkerboard_sdf_path, "-entity","checkerboard_left", 
                   '-x', '0.236','-y', '0.03', '-z', '0.182', '-P', '1.57']
    )

    camera_left_calibrator_cmd = Node(
        package='camera_calibration',
        executable='cameracalibrator',
        name='camera_left_calibrator',
        arguments=["--no-service-check", "-p", "checkerboard", "--size", "8x6",
                     "--square", "0.02", "--ros-args", "-r", "image:=/camera_left/image_raw", "-p", "camera:=/camera_left"]
    )

    ld = LaunchDescription()

    # Add the commands to the launch description
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(urdf_spawner_cmd)
    ld.add_action(checkerboard_left_spawner_cmd)
    ld.add_action(camera_left_calibrator_cmd)

    return ld
