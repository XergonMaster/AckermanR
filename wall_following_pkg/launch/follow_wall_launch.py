from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_name = "wall_following_pkg"
    return LaunchDescription([
        Node(
            package=pkg_name,
            executable='control_node',
            name='control_node',
            parameters=[os.path.join(get_package_share_directory(
                pkg_name), 'config', 'params.yml')],
            output='screen'
        ),
        Node(
            package=pkg_name,
            executable='wall_following_node',
            name='wall_following_node',
            parameters=[os.path.join(get_package_share_directory(
                pkg_name), 'config', 'params.yml')],
            output='screen'
        )
    ])
