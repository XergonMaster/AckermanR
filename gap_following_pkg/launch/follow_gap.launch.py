from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_name = "gap_following_pkg"
    return LaunchDescription([
        Node(
            package=pkg_name,
            executable='follow_gap',
            name='follow_gap',
            parameters=[os.path.join(get_package_share_directory(
                pkg_name), 'config', 'params.yml')],
            output='screen'
        )
    ])
