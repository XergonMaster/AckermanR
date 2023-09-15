import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_wall_following = get_package_share_directory('wall_following_pkg')
    pkg_gap_following = get_package_share_directory('gap_following_pkg')
    # Cambia 'control_car' al nombre real de tu paquete
    pkg_control_car = get_package_share_directory('control_car')

    # wall following
    wall_following = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_wall_following, 'launch', 'follow_wall_launch.py'),
        )
    )

    # gap following
    gap_following = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gap_following, 'launch', 'follow_gap.launch.py'),
        )
    )


    return LaunchDescription([
        wall_following,
        gap_following,
    ])