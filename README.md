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

### Map a world (RTAB-Map, drive with the keyboard)

```shell
bash scripts/build_run_sim_mapping.sh [world] [--new]
```

One map per world is saved in `maps/<world>.db` (gitignored) and continued on the next run; `--new` starts over.
At the end of each run the 2D map is also saved as `maps/<world>.png` + `maps/<world>.yaml` (for the app).
Stop with Ctrl+C in the script's terminal, not with `docker compose down`, otherwise the map may not be saved.

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
