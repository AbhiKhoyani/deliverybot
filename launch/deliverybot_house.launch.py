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

import tempfile
import xacro

def generate_launch_description():
    launch_file_dir = os.path.join(get_package_share_directory('deliverybot'), 'launch')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_realsense2_desc = get_package_share_directory('realsense2_description')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    x_pose = LaunchConfiguration('x_pose', default='-2.0')
    y_pose = LaunchConfiguration('y_pose', default='-0.5')
    z_pose = LaunchConfiguration('y_pose', default='0.0')

    world = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'worlds',
        'turtlebot3_empty.world'
    )

    xacro_path = os.path.join(
        get_package_share_directory('deliverybot'),
        'urdf',
        'deliverybot.urdf.xacro'
    )

    sdf_path = os.path.join(
        get_package_share_directory('deliverybot'),
        'models',
        'model.sdf'
    )
    # urdf_path = tempfile.mktemp(prefix="%s_" % os.path.basename(xacro_path))
    # doc = xacro.process_file(xacro_path)
    # out = xacro.open_output(urdf_path)
    # out.write(doc.toprettyxml(indent='  '))

    # with open(xacro_path, 'r') as infp:
    #     robot_desc = infp.read()

    robot_desc_config = xacro.process_file(xacro_path)
    robot_desc = robot_desc_config.toxml()

    gazebo_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        )
    )
    
    # gzserver_cmd = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
    #     ),
    #     launch_arguments={'world': world}.items()
    # )

    # gzclient_cmd = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
    #     )
    # )

    # robot_state_publisher_cmd = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(launch_file_dir, 'robot_state_publisher.launch.py')
    #     ),
    #     launch_arguments={'use_sim_time': use_sim_time}.items()
    # )

    urdf_spawner_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
        respawn=False,
        arguments=["-file", sdf_path, "-entity","realsense_cam"]
    )

    # urdf_spawner_cmd = Node(
    #     package='realsense2_description',
    #     executable='spawn_robot.py',
    #     name='urdf_spawner',
    #     output='screen',
    #     respawn=False,
    #     arguments=[robot_desc, x_pose, y_pose, z_pose]
    # )

    rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(get_package_share_directory('deliverybot'), 'rviz', 'rviz_config.rviz')]
    )

    robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        # arguments=[urdf_path],
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_desc
            }],
        )
    
    ld = LaunchDescription()

    # Add the commands to the launch description
    ld.add_action(gazebo_cmd)
    # ld.add_action(gzserver_cmd)
    # ld.add_action(gzclient_cmd)
    # ld.add_action(robot_state_publisher_cmd)
    # ld.add_action(rviz_cmd)
    # ld.add_action(urdf_spawner_cmd)

    return ld
