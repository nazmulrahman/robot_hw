#!/usr/bin/python3

from os.path import join
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    hw_pkg = get_package_share_directory('robot_hw')

    # Specify the actions

    launch_motor_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            join(hw_pkg , 'launch' , 'motors.launch.py')
        )
    )

    launch_sensors = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            join(hw_pkg , 'launch' , 'robot_sensors.launch.py')
        )
    )


    # Create the launch description and populate
    ld = LaunchDescription()

    # Add any actions
    ld.add_action(launch_motor_driver)
    ld.add_action(launch_sensors)

    return ld