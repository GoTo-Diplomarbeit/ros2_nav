"""
Mapping and navigation with RTAB-Map instead of slam_toolbox.

This is a drop-in alternative to navigation.launch.py. The difference:

  navigation.launch.py          3D Velodyne cloud -> pointcloud_to_laserscan -> flat
                                /scan -> slam_toolbox -> 2D /map only.

  navigation_rtabmap.launch.py  3D Velodyne cloud -> RTAB-Map -> a 3D map (OctoMap on
                                /octomap_occupied_space, raw cloud on /cloud_map) AND a
                                2D /map projected out of it for Nav2.
                                pointcloud_to_laserscan still runs (with Nav2 only), for
                                Nav2's live obstacle checks (/scan) - not for mapping.

Why rviz shows the OctoMap and not /cloud_map: the Go2's trot rocks its body about
+-7 deg, so some floor hits get placed in the air. /cloud_map keeps every point it
ever stored, those "streaks" included. The OctoMap ray-traces in 3D and erases a wrong
point when a later beam passes through it (tested: 0.2 % wrong points vs 1.8 %).

Nav2 is unchanged and needs no edits: RTAB-Map publishes the same nav_msgs/OccupancyGrid
on /map and the same map -> odom transform that slam_toolbox did. It simply drops into
the same slot in the TF tree.

Easiest: scripts/run_sim_mapping.sh (mapping, use_nav2:=false + teleop) or
scripts/run_sim_navigation.sh (localization on a saved map + Nav2) start the simulator
and this file in the right order. By hand, after the simulator is running:
    ros2 launch unitree_go2_sim unitree_go2_launch.py static_map_to_odom:=false rviz:=false
    ros2 launch goto navigation_rtabmap.launch.py

Things that will bite you if you skip them:

  1. The simulator must NOT publish its own map -> odom transform, exactly as with
     slam_toolbox - two publishers for one frame breaks TF. Always start it with
     static_map_to_odom:=false.

  2. Never run this at the same time as navigation.launch.py. Both slam_toolbox and
     RTAB-Map publish map -> odom, which is the same conflict again.

  3. The map is saved in database_path (default ~/.ros/rtabmap.db; all scans + poses,
     the 2D and 3D maps can be rebuilt from it). The default delete_db:=true DELETES it
     at startup so every run maps from scratch - copy it away first to keep it.
     scripts/run_sim_mapping.sh does NOT use these defaults: it keeps one map per
     Gazebo world in ~/maps/<world>.db and continues it on every start.

  3b. Continuing an existing map: start with delete_db:=false localization:=true.
     RTAB-Map then only LOCATES the robot on the loaded map (no new map nodes). Once it
     has found the robot (first message on /localization_pose), switch to mapping:
         ros2 service call /rtabmap/set_mode_mapping std_srvs/srv/Empty
     New areas are then added to the SAME map. Why not start in mapping mode directly:
     a new run in mapping mode begins a separate, unconnected map, and without a camera
     RTAB-Map has no way to find the old map again - the old map would disappear.

  4. To watch the 3D map build live, start the simulator with rviz:=false and let this
     launch own rviz instead (rviz:=true, the default here). The simulator's own rviz
     config has no 3D map display, so two rviz windows would just mean one useless one.

  5. Known issue: occasionally RTAB-Map starts, receives every scan but processes none
     (no /info, empty map; cause not found yet). Check with "ros2 topic hz /info"
     (~1 Hz when working) and restart this launch if it stays silent.
     scripts/run_sim_mapping.sh detects and restarts it automatically.
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
        description="Delete the map file (database_path) at startup so the run maps from scratch. "
        "Set to false to extend the map from a previous run.",
    )

    database_path_argument = DeclareLaunchArgument(
        "database_path",
        default_value="~/.ros/rtabmap.db",
        description="The map file RTAB-Map reads and writes.",
    )

    localization_argument = DeclareLaunchArgument(
        "localization",
        default_value="false",
        description="true = only locate the robot on the loaded map, don't change the map "
        "(Mem/IncrementalMemory=false). Switch to mapping at runtime with the "
        "service /rtabmap/set_mode_mapping.",
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

    # --delete_db_on_start is a command line flag, not a ROS parameter, so it has to be
    # built as a conditional argument string rather than passed in the parameter file.
    delete_database_flag = PythonExpression(
        ["'--delete_db_on_start' if '", LaunchConfiguration("delete_db"), "' == 'true' else ''"]
    )

    # Settings that come from the launch arguments, on top of rtabmap_params.yaml.
    # ParameterValue(..., value_type=str): RTAB-Map's own parameters must be STRINGS -
    # without it, launch would turn the text "false" into a boolean and RTAB-Map would
    # reject the parameter.
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

    # Flat 2D /scan cut out of the 3D Velodyne cloud (only points 0.1 - 0.5 m above the
    # floor). RTAB-Map itself does not need it, but Nav2 does: the collision monitor
    # (last safety stage before /cmd_vel) and the costmaps' obstacle layers read /scan.
    # Without it the collision monitor sees no sensor data, assumes it is blind and
    # blocks every movement - the robot never moves to a "2D Goal Pose".
    # Same settings as in navigation.launch.py (the slam_toolbox pipeline).
    # Only started together with Nav2 (use_nav2) - nothing else reads /scan.
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

    # RTAB-Map's own viewer (off by default). It gets the same parameter file so it
    # uses the same frames and inputs - the file's "/**" key applies it to both nodes.
    rtabmap_viewer_node = Node(
        package="rtabmap_viz",
        executable="rtabmap_viz",
        name="rtabmap_viz",
        output="screen",
        parameters=[rtabmap_parameters_file, simulation_time_parameter, rtabmap_launch_parameters],
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
