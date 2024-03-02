# Dunder Mifflin, Inc.
# Author: Michel Scott

# System imports
from distutils.command.config import config
import os
import sys
import xacro
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
    
    mechanical_odometry_node = Node(package='uiabot',
                                    namespace=namespace,
                                    executable='mechanical_odometry',
                                    parameters=[{'use_tf': False}])
    
    # Include odrive_ros2 executable
    odrive_ros2_node = Node(package='odrive_ros2',
                            namespace=namespace,
                            executable='odrive_ros2')

    # Include BNO055 IMU launch description
    bno055_node = Node(package='bno055_i2c_ros2',
                        namespace=namespace,
                        executable='bno055_i2c_ros2')

    # Include EKF node to fuse odometry
    ekf_params = os.path.join(uiabot_shared, "params/ekf_params.yaml")
    ekf_node = Node(package='robot_localization',
                    namespace=namespace,
                    executable='ekf_node',
                    name='ekf_filter_node',
                    output='screen',
                    parameters=[os.path.join(uiabot_shared, ekf_params)])

    # Include lidar launch description
    rplidar_node = Node(name='rplidar_composition',
                        package='rplidar_ros',
                        executable='rplidar_composition',
                        output='screen',
                        parameters=[{
                            'serial_port': '/dev/ttyUSB0',
                            'serial_baudrate': 115200,
                            'frame_id': 'laser',
                            'inverted': False,
                            'angle_compensate': True}])

    # Include slam launch description
    slam_shared = FindPackageShare('slam_toolbox').find('slam_toolbox')
    slam_launch = IncludeLaunchDescription(
                                    PythonLaunchDescriptionSource(os.path.join(slam_shared, 'launch', 'online_async_launch.py')))

    # Set up robot state publisher
    uiabot_xacro_path = 'urdf/uiabot.urdf.xacro'
    uiabot_xacro_abs_path = FindPackageShare('uiabot').find('uiabot') + '/' + uiabot_xacro_path
    uiabot_urdf = xacro.process_file(uiabot_xacro_abs_path)
    robot_state_publisher_node = Node(package='robot_state_publisher',
                                      namespace=namespace,
                                      executable='robot_state_publisher',
                                      parameters=[{'robot_description': uiabot_urdf.toxml(),
                                                   'use_sim_time': False}])
    
    # Instantiate launch description
    ld = LaunchDescription()

    # Add nodes to launch description
    ld.add_action(control_node)
    ld.add_action(mechanical_odometry_node)
    ld.add_action(odrive_ros2_node)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(bno055_node)
    ld.add_action(ekf_node)
    ld.add_action(rplidar_node)
    ld.add_action(slam_launch)

    return ld