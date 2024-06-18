#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    self_dir = get_package_share_directory('deliverybot')

    deliverybot_bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(self_dir, 'launch', 'deliverybot_house.launch.py')
        )
    )

    rtabmap_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(self_dir, 'launch', 'rtabmap.launch.py')
        )
    )

    ld = LaunchDescription()

    # Add the commands to the launch description
    ld.add_action(deliverybot_bringup_cmd)
    ld.add_action(rtabmap_cmd)

    return ld
