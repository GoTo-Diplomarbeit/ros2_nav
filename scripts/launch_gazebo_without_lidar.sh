cd ~/

# "$@" passes on any extra arguments, e.g.: scripts/launch_gazebo_without_lidar.sh world:=school_with_person
ros2 launch unitree_go2_sim unitree_go2_launch.py disable_velodyne_lidar:=true disable_d455:=true rviz:=false "$@"
