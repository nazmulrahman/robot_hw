#!/usr/bin/python3

from os.path import join
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    hw_pkg = get_package_share_directory('robot_hw')
    hw_params_file = LaunchConfiguration('hw_params_file')

    #Launch Arguments

    declare_params_file_cmd = DeclareLaunchArgument(
        name='hw_params_file',
        default_value=join(hw_pkg, 'config', 'hw_config.yaml'),
        description= 'Used as default hardware configuration if not provided through bringup package'
    )


    # Specify the actions

    start_imu_publisher = Node(
        package='robot_hw',
        executable='imu_publisher',
        parameters=[hw_params_file],
    )


    # Create the launch description and populate
    ld = LaunchDescription()
    ld.add_action(declare_params_file_cmd)

    # Add any actions
    ld.add_action(start_imu_publisher)


    return ld