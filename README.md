# How to install this project
## First install prerequisites
### First install Ubuntu 24.04 LTS either on a laptop or on wsl2
#### For WSL:
```bash
wsl --install -d Ubuntu-24.04
```

### Clone the repository to Documents
```bash
git clone https://github.com/Humanoiderdepp/ros2_nav.git /home/gazebo/Documents
```

### Run the setup.sh script

```bash
cd ~/Documents/ros2_nav/
chmod +x setup.sh

./setup.sh
```

# Choosing a Gazebo world (map)

The simulator can start with different Gazebo worlds. The worlds are `.sdf` files in the
simulator repository (cloned by `setup.sh` to `~/go2_ws/src/unitree_go2_ros2`):

```
~/go2_ws/src/unitree_go2_ros2/unitree_go2_description/worlds/
```

| World                | What it contains                                                    |
|----------------------|---------------------------------------------------------------------|
| `default`            | School entrance mockup (used when you don't pick a world)           |
| `school_with_person` | Same school mockup + a standing person 2 m in front of the robot    |

## Start the simulator with one world

Pass the world's file name (without `.sdf`) as `world:=`:

```bash
ros2 launch unitree_go2_sim unitree_go2_launch.py world:=school_with_person
```

The helper scripts take the same argument:

```bash
# Simulator only
scripts/launch_gazebo_with_lidar.sh world:=school_with_person

# Simulator + SLAM + Nav2 + teleop: first argument = SLAM backend, second = world
scripts/run_sim_mapping.sh rtabmap school_with_person
```

A full path to any `.sdf` file works too: `world:=/home/<you>/my_world.sdf`.

## Add a new world

1. Copy an existing world in the folder above, e.g. `cp default.sdf my_world.sdf`, and edit it.
2. Rebuild the simulator workspace, otherwise the new file is not found:
   ```bash
   cd ~/go2_ws && PYTHONNOUSERSITE=1 colcon build --symlink-install --packages-select unitree_go2_description
   ```
3. Start it with `world:=my_world`.

Pitfalls:
- If you misspell the name, the launch stops and prints the list of available worlds.
- The robot always spawns at x=0, y=0 facing +x. Keep that spot free in new worlds
  (or move the robot with `world_init_x:=`, `world_init_y:=`, `world_init_heading:=`).
- `school_with_person` downloads the person model from Gazebo Fuel on its first start,
  so that first start needs internet. After that it is cached in `~/.gz/fuel`.
