import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('go2_nav'), 'launch', 'navigation.launch.py')
        ),
    )

    return LaunchDescription([
        navigation_launch,
        Node(
            package='go2_nav',
            executable='movement_node',
            name='movement',
            output='screen',
        ),
        Node(
            package='go2_nav',
            executable='vision_node',
            name='vision',
            output='screen',
        ),
        Node(
            package='go2_nav',
            executable='planner_node',
            name='planner',
            output='screen',
        ),
        Node(
            package='go2_nav',
            executable='camera_node',
            name='camera',
            output='screen',
        ),
        Node(
            package='image_transport',
            executable='republish',
            name='image_republisher',
            arguments=['raw', 'compressed'],
            remappings=[
                ('in', '/rgb_image'),
                ('out/compressed', '/rgb_image/compressed'),
            ],
            output='screen',
        ),
    ])
