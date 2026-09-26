# Unitree Go2 — ROS 2 simulation examples

Small ROS 2 Jazzy examples that drive the simulated Unitree Go2 in Gazebo. The
whole movement interface is one topic, `/cmd_vel` (`geometry_msgs/Twist`).

## Requirements

- **Ubuntu 24.04** with **ROS 2 Jazzy** and the built Go2 overlay in `~/go2_ws`.
- **Windows:** use the provided WSL distribution instead of installing anything.
  It already contains Ubuntu-24.04, ROS 2 Jazzy, `~/go2_ws` and the PyCharm
  interpreter at `~/venvs/go2`. Import it and work inside WSL — keep the default
  `gazebo` user, because the venv has `/home/gazebo/...` paths baked into it.

## Run

Terminal 1 — start the simulator:

```bash
ros2 launch unitree_go2_sim unitree_go2_launch.py
```

Wait ~30 s until the controllers are active (`ros2 control list_controllers` —
`joint_group_effort_controller` must say `active`); commands sent earlier are
ignored.

Terminal 2 — from this directory:

```bash
./run.sh                            # runs movement/MovementNode.py
./run.sh movement/other.py  # any other script in this repo
source ./run.sh                     # environment only, runs nothing
```

`scripts/run.sh` sources `/opt/ros/jazzy/setup.bash` and `~/go2_ws/install/setup.bash`,
then runs the script with `python3`. If your layout differs, override
`ROS_SETUP`, `WS_SETUP` or `PYTHON`. Ctrl-C stops the robot cleanly — the example
publishes zero velocity before shutting down.

## Layout

| Path | What it is |
|---|---|
| `scripts/run.sh` | Sources the ROS 2 + Go2 environment, then runs a script. |
| `movement/MovementNode.py` | The example: a forward `Twist` on `/cmd_vel`. |
| `movement/README.md` | How it works, measured results, PyCharm setup, troubleshooting. |
| `scripts/setup-venv.sh` | PyCharm/WSL only: builds the interpreter the IDE uses. |
| `.editorconfig` | Keeps `.sh` / `.py` at LF, so Windows editors cannot break the scripts. |

## PyCharm

See [unitree_movement/README.md](movement/README.md#pycharm-setup) —
interpreter setup, why the venv needs a wrapper, and the known failure modes.
