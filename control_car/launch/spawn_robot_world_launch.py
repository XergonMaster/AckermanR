import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_sim_car = get_package_share_directory('sim_car')
    # Cambia 'control_car' al nombre real de tu paquete
    pkg_control_car = get_package_share_directory('control_car')

    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py'),
        )
    )

    car = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_sim_car, 'launch', 'spawn_car.launch.py'),
        )
    )

    # Llama al lanzamiento del mux de cmd_vel

    cmd_vel_mux = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_control_car, 'launch', 'cmd_vel_mux_launch.py'),
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value=[os.path.join(
                pkg_control_car, 'world', 'Parcial_World_V2-1.world'), ''],
            description='SDF world file'),
        gazebo,
        car,
        cmd_vel_mux,
    ])