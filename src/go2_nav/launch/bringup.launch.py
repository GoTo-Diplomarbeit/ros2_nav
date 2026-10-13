"""
Starts the project's own nodes (movement, vision, planner) plus mapping + navigation.

Launch arguments:
    backend:=gazebo|go2        where movement commands go (simulator or real robot)
    network_interface:=eth0    network interface of the real robot (backend:=go2 only)
    slam:=slam_toolbox|rtabmap which SLAM to use (default slam_toolbox):
                                 slam_toolbox -> navigation.launch.py (2D map)
                                 rtabmap      -> navigation_rtabmap.launch.py (3D + 2D map)

The simulator is NOT started here - start it first, with static_map_to_odom:=false.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    launch_directory = os.path.join(get_package_share_directory('go2_nav'), 'launch')

    # Exactly ONE of these may run: both slam_toolbox and RTAB-Map publish the
    # map -> odom transform, and a frame with two publishers breaks TF.
    slam_backend = LaunchConfiguration("slam")

    slam_toolbox_navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_directory, 'navigation.launch.py')),
        condition=IfCondition(PythonExpression(["'", slam_backend, "' == 'slam_toolbox'"])),
    )

    rtabmap_navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_directory, 'navigation_rtabmap.launch.py')),
        condition=IfCondition(PythonExpression(["'", slam_backend, "' == 'rtabmap'"])),
    )

    return LaunchDescription([
        DeclareLaunchArgument("backend", default_value="gazebo", description="Movement backend: gazebo or go2"),
        DeclareLaunchArgument("network_interface", default_value="eth0",
                              description="Network interface of the real robot (backend:=go2 only)"),
        DeclareLaunchArgument("slam", default_value="slam_toolbox",
                              description="SLAM backend: slam_toolbox (2D map) or rtabmap (3D map + 2D grid)"),
        slam_toolbox_navigation_launch,
        rtabmap_navigation_launch,
        Node(
            package='go2_nav',
            executable='movement_node',
            name='movement',
            output='screen',
            parameters=[{"backend": LaunchConfiguration("backend"),
                         "network_interface": LaunchConfiguration("network_interface")}],
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
