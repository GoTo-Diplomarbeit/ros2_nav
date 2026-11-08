"""
Mapping and navigation with RTAB-Map - the alternative to navigation.launch.py (slam_toolbox).

  navigation.launch.py          Velodyne -> pointcloud_to_laserscan -> /scan -> slam_toolbox -> 2D /map
  navigation_rtabmap.launch.py  Velodyne -> RTAB-Map -> 3D map (OctoMap) + 2D /map for Nav2

Nav2 needs no changes: RTAB-Map publishes the same /map and map -> odom transform as slam_toolbox.
rviz shows the OctoMap instead of /cloud_map: the Go2's trot rocks the lidar by about 7 degrees,
and only the OctoMap erases the wrong points this causes once a later beam passes through them.

Usually started by scripts/build_run_sim_mapping.sh or build_run_sim_navigation.sh, which keep
one map per world in maps/<world>.db. By hand, after the simulator:
    ros2 launch unitree_go2_sim unitree_go2_launch.py static_map_to_odom:=false rviz:=false
    ros2 launch goto navigation_rtabmap.launch.py

Pitfalls:
  - By default the map file (database_path) is deleted at startup. Pass delete_db:=false to keep it.
  - map -> odom may only have one publisher: start the simulator with static_map_to_odom:=false,
    and never run this together with navigation.launch.py.
  - Continuing a saved map: delete_db:=false localization:=true, and once the robot is found
    (/localization_pose), call the service /rtabmap/set_mode_mapping. Starting in mapping mode
    instead would begin a new map that isn't connected to the old one.
  - Sometimes RTAB-Map starts but processes no scans (no /info, cause unknown) - restart it then.
    The scripts do that automatically.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    goto_share_directory = get_package_share_directory("goto")
    rtabmap_parameters_file = os.path.join(goto_share_directory, "config", "rtabmap_params.yaml")
    nav2_parameters_file = os.path.join(goto_share_directory, "config", "nav2_params.yaml")

    simulation_time_parameter = {"use_sim_time": True}

    delete_database_argument = DeclareLaunchArgument(
        "delete_db",
        default_value="true",
        description="Delete the map file at startup, so the run maps from scratch (never with localization:=true).",
    )

    database_path_argument = DeclareLaunchArgument(
        "database_path",
        default_value="~/.ros/rtabmap.db",
        description="The map file RTAB-Map loads and saves.",
    )

    localization_argument = DeclareLaunchArgument(
        "localization",
        default_value="false",
        description="true = only find the robot on the loaded map, add nothing to it. "
        "Switch to mapping later with the service /rtabmap/set_mode_mapping.",
    )

    use_navigation_argument = DeclareLaunchArgument(
        "use_nav2",
        default_value="true",
        description="Also start Nav2. Set to false to map by teleop only, which leaves far more CPU for RTAB-Map.",
    )

    use_viewer_argument = DeclareLaunchArgument(
        "rtabmap_viz",
        default_value="false",
        description="Start RTAB-Map's own 3D viewer. It shows the live 3D cloud, the pose "
        "graph and loop closures, but it is expensive - leave it off on a "
        "throttled laptop and use rviz instead.",
    )

    use_rviz_argument = DeclareLaunchArgument(
        "rviz",
        default_value="true",
        description="Start rviz2 with the 3D mapping config (rviz/rtabmap.rviz). "
        "Launch the simulator with rviz:=false so there is only one window.",
    )

    # --delete_db_on_start is a command line flag, not a ROS parameter. Never delete in localization
    # mode - that would leave nothing to localize on.
    delete_database_flag = PythonExpression(
        [
            "'--delete_db_on_start' if '",
            LaunchConfiguration("delete_db"),
            "' == 'true' and '",
            LaunchConfiguration("localization"),
            "' != 'true' else ''",
        ]
    )

    # Settings from the launch arguments, on top of rtabmap_params.yaml. value_type=str: RTAB-Map's
    # own parameters must be strings, otherwise launch turns "false" into a boolean and RTAB-Map rejects it.
    rtabmap_launch_parameters = {
        "database_path": LaunchConfiguration("database_path"),
        "Mem/IncrementalMemory": ParameterValue(
            PythonExpression(["'false' if '", LaunchConfiguration("localization"), "' == 'true' else 'true'"]),
            value_type=str,
        ),
    }

    # The SLAM node itself. "scan_cloud" is RTAB-Map's generic 3D input topic, which we
    # point at the Velodyne. Odometry is not remapped because rtabmap_params.yaml sets
    # odom_frame_id, which makes RTAB-Map read odometry from TF instead of a topic.
    rtabmap_node = Node(
        package="rtabmap_slam",
        executable="rtabmap",
        name="rtabmap",
        output="screen",
        parameters=[rtabmap_parameters_file, simulation_time_parameter, rtabmap_launch_parameters],
        remappings=[("scan_cloud", "/velodyne_points/points")],
        arguments=[delete_database_flag],
    )

    # Flat /scan (points 0.1 - 0.5 m above the floor) for Nav2 only: without it the collision monitor
    # sees no sensor data and blocks every movement. Same settings as in navigation.launch.py.
    pointcloud_to_laserscan_node = Node(
        package="pointcloud_to_laserscan",
        executable="pointcloud_to_laserscan_node",
        remappings=[("cloud_in", "/velodyne_points/points"), ("scan", "/scan")],
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
        condition=IfCondition(LaunchConfiguration("use_nav2")),
    )

    # RTAB-Map's own viewer (off by default), with the same parameter file for the same frames and inputs
    rtabmap_viewer_node = Node(
        package="rtabmap_viz",
        executable="rtabmap_viz",
        name="rtabmap_viz",
        output="screen",
        parameters=[rtabmap_parameters_file, simulation_time_parameter],
        remappings=[("scan_cloud", "/velodyne_points/points")],
        condition=IfCondition(LaunchConfiguration("rtabmap_viz")),
    )

    # rviz2 showing the live 3D map (OctoMap), the 2D grid (/map) and the current
    # Velodyne sweep. use_sim_time matters: without it rviz compares simulation
    # timestamps against the wall clock and drops every message as "too old".
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", os.path.join(goto_share_directory, "rviz", "rtabmap.rviz")],
        parameters=[simulation_time_parameter],
        condition=IfCondition(LaunchConfiguration("rviz")),
    )

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("nav2_bringup"), "launch", "navigation_launch.py")
        ),
        launch_arguments={"use_sim_time": "true", "params_file": nav2_parameters_file}.items(),
        condition=IfCondition(LaunchConfiguration("use_nav2")),
    )

    return LaunchDescription(
        [
            delete_database_argument,
            database_path_argument,
            localization_argument,
            use_navigation_argument,
            use_viewer_argument,
            use_rviz_argument,
            rtabmap_node,
            pointcloud_to_laserscan_node,
            rtabmap_viewer_node,
            rviz_node,
            navigation_launch,
        ]
    )
