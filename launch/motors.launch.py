#!/usr/bin/python3

from os.path import join
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_share = get_package_share_directory('robot_hw')
    hw_params_file = LaunchConfiguration('hw_params_file')

    #Launch Arguments

    declare_params_file_cmd = DeclareLaunchArgument(
        name='hw_params_file',
        default_value=join(pkg_share, 'config', 'hw_config.yaml'),
        description= 'Used as default hardware configuration if not provided through bringup package'
    )


    # Specify the actions

    start_motor_control = Node(
        package='robot_hw',
        executable='motor_controller_pid',
        parameters=[hw_params_file],
    )

    start_encoder_publisher = Node(
        package='robot_hw',
        executable='encoder_publisher',
        parameters=[hw_params_file],
    )

    start_odom_publisher = Node(
        package='robot_hw',
        executable='odom_publisher_node',
        parameters=[hw_params_file],
    )




    # Create the launch description and populate
    ld = LaunchDescription()
    ld.add_action(declare_params_file_cmd)

    # Add any actions
    ld.add_action(start_motor_control)
    ld.add_action(start_encoder_publisher)
    ld.add_action(start_odom_publisher)

    return ld