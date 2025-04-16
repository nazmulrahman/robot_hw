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
    lidar_pkg = get_package_share_directory('rplidar_ros')
    rtk_pkg = get_package_share_directory('ublox_gps')


    # Specify the actions

    launch_rtk = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            join(rtk_pkg , 'launch' , 'ublox_gps_node-launch.py')
        )
    )

    start_rtk_fix_publisher = Node(
        package='rtk_python',
        executable='rtk_fix_pub',
    )


    launch_lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            join(lidar_pkg , 'launch' , 'rplidar_s2_launch.py')
        )
    )

    launch_imu = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            join(hw_pkg , 'launch' , 'imu.launch.py')
        )
    )



    # Create the launch description and populate
    ld = LaunchDescription()

    # Add any actions
    ld.add_action(launch_rtk)
    ld.add_action(start_rtk_fix_publisher)
    ld.add_action(launch_lidar)
    ld.add_action(launch_imu)

    return ld