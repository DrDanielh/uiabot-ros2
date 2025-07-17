# System imports
from distutils.command.config import config
import os
import sys
import logging

# Ros launch imports
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    # Parameters
    namespace = ""

    # Instantiate logger
    logger = logging.getLogger('launch')

    # Uiabot package share
    uiabot_shared = FindPackageShare('uiabot').find('uiabot') 
   
    # Include uiabot executables
    control_node = Node(package='uiabot',
                        namespace=namespace,
                        executable='control')

    # Include odrive_ros2 executable
    odrive_ros2_node = Node(package='odrive_ros2',
                            namespace=namespace,
                            executable='odrive_ros2')

    # Instantiate launch description
    ld = LaunchDescription()

    # Add nodes to launch description
    ld.add_action(control_node)
    ld.add_action(odrive_ros2_node)

    return ld