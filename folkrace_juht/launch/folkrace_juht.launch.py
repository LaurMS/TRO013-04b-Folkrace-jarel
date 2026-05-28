"""Käivitusfail: Webots + folkrace_juht.

Käivita: ros2 launch folkrace_juht folkrace_juht.launch.py
"""
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    webots_pkg = get_package_share_directory('yahboom_webots')

    webots_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(webots_pkg, 'launch', 'webots.launch.py')
        ),
    )

    folkrace_node = Node(
        package='folkrace_juht',
        executable='folkrace_juht',
        name='folkrace_juht',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    return LaunchDescription([
        webots_launch,
        folkrace_node,
    ])
