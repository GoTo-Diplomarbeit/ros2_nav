# "$@" passes on any extra arguments, e.g.: scripts/launch_gazebo_with_lidar.sh world:=school_with_person
ros2 launch unitree_go2_sim unitree_go2_launch.py static_map_to_odom:=false "$@"
