from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
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
    ])
