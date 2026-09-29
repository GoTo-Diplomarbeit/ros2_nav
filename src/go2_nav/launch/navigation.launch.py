import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    simulation_time_parameter = {"use_sim_time": True}

    pointcloud_to_laserscan_node = Node(
        package="pointcloud_to_laserscan",
        executable="pointcloud_to_laserscan_node",
        remappings=[("cloud_in", "/unitree_lidar/points"), ("scan", "/scan")],
        parameters=[{"target_frame": "base_link", "min_height": 0.1, "max_height": 0.5}, simulation_time_parameter],
    )

    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("slam_toolbox"), "launch", "online_async_launch.py")
        ),
        launch_arguments={"use_sim_time": "true"}.items(),
    )

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("nav2_bringup"), "launch", "navigation_launch.py")
        ),
        launch_arguments={"use_sim_time": "true"}.items(),
    )

    return LaunchDescription([pointcloud_to_laserscan_node, slam_toolbox_launch, navigation_launch])
