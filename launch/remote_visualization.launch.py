#!/usr/bin/env python3
"""
Launch file for visualization on remote PC.
This should be run on the remote PC while the robot runs teleop_mechanical_odometry.launch.py
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the package share directory
    uiabot_share = get_package_share_directory('uiabot')
    
    # Path to the RViz config file
    rviz_config_file = os.path.join(uiabot_share, 'rviz', 'teleop_mechanical_odometry.rviz')
    
    # RViz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )
    
    return LaunchDescription([
        rviz_node
    ])