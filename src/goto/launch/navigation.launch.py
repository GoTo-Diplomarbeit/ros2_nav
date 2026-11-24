import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    goto_share_directory = get_package_share_directory("goto")
    simulation_time_parameter = {"use_sim_time": True}

    # The defaults keep bringup.launch.py as it was. The slam_toolbox autonomous mapping script sets all four.
    nav2_params_argument = DeclareLaunchArgument(
        "nav2_params",
        default_value=os.path.join(goto_share_directory, "config", "nav2_params.yaml"),
        description="The Nav2 parameter file (the exploration script passes a merged copy).",
    )
    slam_params_argument = DeclareLaunchArgument(
        "slam_params",
        default_value=os.path.join(goto_share_directory, "config", "slam_params.yaml"),
        description="The slam_toolbox parameter file (the exploration script passes a merged copy).",
    )
    use_rviz_argument = DeclareLaunchArgument(
        "rviz", default_value="false", description="Also start rviz2. Launch the simulator with rviz:=false then."
    )
    rviz_config_argument = DeclareLaunchArgument(
        "rviz_config", default_value="autonomous_mapping.rviz", description="The rviz config in rviz/."
    )

    pointcloud_to_laserscan_node = Node(
        package="pointcloud_to_laserscan",
        executable="pointcloud_to_laserscan_node",
        remappings=[("cloud_in", "/velodyne_points/points"), ("scan", "/scan_raw")],
        parameters=[
            {
                "target_frame": "base_footprint",
                "min_height": 0.1,
                "max_height": 0.5,
                "range_max": 8.0,
                "transform_tolerance": 0.2,
            },
            simulation_time_parameter,
        ],
    )

    # /scan_raw -> /scan without the robot's own legs (they made the collision monitor block the robot)
    scan_body_filter_node = Node(package="goto", executable="scan_body_filter")

    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("slam_toolbox"), "launch", "online_async_launch.py")
        ),
        launch_arguments={
            "use_sim_time": "true",
            "slam_params_file": LaunchConfiguration("slam_params"),
        }.items(),
    )

    # use_sim_time: without it rviz drops every message as too old
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", PathJoinSubstitution([goto_share_directory, "rviz", LaunchConfiguration("rviz_config")])],
        parameters=[simulation_time_parameter],
        condition=IfCondition(LaunchConfiguration("rviz")),
    )

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("nav2_bringup"), "launch", "navigation_launch.py")
        ),
        launch_arguments={
            "use_sim_time": "true",
            "params_file": LaunchConfiguration("nav2_params"),
        }.items(),
    )

    return LaunchDescription(
        [
            nav2_params_argument,
            slam_params_argument,
            use_rviz_argument,
            rviz_config_argument,
            pointcloud_to_laserscan_node,
            scan_body_filter_node,
            slam_toolbox_launch,
            rviz_node,
            navigation_launch,
        ]
    )
