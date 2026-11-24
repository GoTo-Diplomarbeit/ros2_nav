# GoTo

## Prerequisites

Install Docker Compose (rooted):

```shell
bash scripts/setup/install-docker.sh
```

Install hooks:

```shell
bash scripts/setup/install-hooks.sh
```

NVIDIA GPU only - install the NVIDIA Container Toolkit, then use `release-nvidia` instead of `release` (scripts: `SERVICE=release-nvidia bash scripts/...` or the `(nvidia)` run configs):

```shell
bash scripts/setup/install-nvidia.sh
```

## Installation

```shell
docker compose pull release
```

## Verify

```shell
docker compose up -d release
docker compose ps
```

The `release` container should show as `running`.

## Run

### Build and run project

```shell
bash scripts/build_run_project.sh
```

### Launch Gazebo

```shell
bash scripts/launch_gazebo.sh
```

On Arch/CachyOS Gazebo's GUI may open blank and log `Requesting list of world names` forever, because
gz-transport discovers the server via multicast and Arch has no multicast route on the loopback interface.
The container therefore sets `GZ_IP=127.0.0.1` (see `docker-compose.yaml`), so gz-transport uses loopback
instead - no host changes needed. After updating, recreate the container once (`docker compose up -d`).

### Map a world (RTAB-Map, drive with the keyboard)

```shell
bash scripts/build_run_sim_mapping.sh [world] [--new]
```

One map per world is saved in `maps/<world>.db` (gitignored) and continued on the next run; `--new` starts over.
At the end of each run the 2D map is also saved as `maps/<world>.png` + `maps/<world>.yaml` (for the app).
Stop with Ctrl+C in the script's terminal, not with `docker compose down`, otherwise the map may not be saved.

### Map a world on its own (slam_toolbox, autonomous)

```shell
bash scripts/build_run_sim_autonomous_mapping_slam_toolbox.sh [world]
```

The robot explores the floor by itself: it drives to the edge of the known map (a "frontier") again and again,
until none is left. slam_toolbox builds the map, Nav2 drives, `autonomous_mapping_node` chooses where to go. Worlds:
`default`, `school_floor` (a floor with corridors, classrooms and tables). `GUI=false` starts Gazebo without its
window (less CPU).

Keys in the script's terminal (you can take over at any time):

| Key | Does |
|---|---|
| `p` | pause / resume exploring |
| `i` `j` `l` `,` | drive by hand (pauses exploring first) |
| `k` | stop moving |
| `x` or Ctrl+C | stop and save the map |

The robot stops and waits when no frontier is left (`done`), when frontiers are left that it can't reach (`stuck`),
or after 30 minutes (`time_limit`). Then press `x` to save, or drive to a missed area by hand and press `p` to
continue.

Result in `maps/` (always a new map; an older one is moved to `<world>_slam_toolbox_old_<date>.*`):
`<world>_slam_toolbox.png` + `.yaml` is the 2D map, `.posegraph` + `.data` is slam_toolbox's own file. Logs are in
`log/sim_runs/<date>/`: `explorer.log` (what the explorer did), `drift.csv` + `drift_summary.txt` (how far the SLAM
pose was from the real one in the simulator) and `cut_off_*.npz` (the costmap around the robot whenever it was
cut off from all frontiers).

While exploring the robot drives backwards when the path leads behind it, and Nav2 uses a wider safety margin around
walls (`config/nav2_exploration_overrides.yaml`). Normal navigation keeps the plain `config/nav2_params.yaml`.

`scripts/build_run_sim_autonomous_mapping.sh` is the same with RTAB-Map. It is not maintained any more and not
tested with the latest changes.

### Navigate on a saved map (RTAB-Map + Nav2)

```shell
bash scripts/build_run_sim_navigation.sh [world]
```

Set goals in rviz with "2D Goal Pose". Logs of both scripts are in `log/sim_runs/`.

### Define rooms (for the app's room selection)

```shell
bash scripts/build_run_define_rooms.sh [world]
```

Opens the map image of a mapped world. For each room: type its name in the terminal, click its corners in the
window, then where the robot should stop and a point in the direction it should face. The rooms are saved in
`maps/<world>_rooms.yaml` (in metres on the map). After continuing a map, check that the rooms still fit.

**Rooms from the app:** the app sends all rooms of the map as a JSON list, in metres on the map, with the
same fields as the room file (`yaw` = heading at the goal in radians, 0 = along the map's x axis):

```json
[{"name": "1.12", "area": [[2.0, 1.0], [6.0, 1.0], [6.0, 4.5]], "goal": {"x": 3.2, "y": 0.6, "yaw": 1.571}}]
```

`web_server_node` saves them with `save_rooms(path, rooms_from_app(text))` from `goto/rooms/room_file.py`. The
list replaces the file (a room left out is deleted). Bad data raises a `ValueError` with a clear message - answer it
with `400 Bad Request`. The planner reads the file for every trip, so new rooms work without a restart.

### Drive to a room (planner_node)

While `build_run_sim_navigation.sh` runs, send a room name from the world's room file:

```shell
ros2 topic pub --once -w 1 /goto_room std_msgs/String "data: '1.12'"   # -w 1: wait for the planner, or the message can get lost
ros2 topic echo /navigation_status
```

**Interface for the app** (`web_server_node`):

| Topic | Type | Direction | Content |
|---|---|---|---|
| `/goto_room` | `std_msgs/String` | app → planner | room name as in `maps/<world>_rooms.yaml`; an empty string stops the trip |
| `/navigation_status` | `std_msgs/String` (JSON) | planner → app | `{"room": "1.12", "status": "driving", "distance_left": 3.2}`; status: `driving`, `arrived`, `failed`, `canceled`, `unknown_room`, `nav2_not_ready` |

`/navigation_status` keeps its last message (`TRANSIENT_LOCAL`), so a late subscriber gets it at once.
