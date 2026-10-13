#!/bin/bash
# Setup for the Go2 navigation project (Ubuntu 24.04, ROS 2 Jazzy).
#
# Run it from inside your clone of the project:
#   git clone https://github.com/Humanoiderdepp/ros2_nav.git
#   cd ros2_nav
#   ./setup.sh
#
# The clone itself is the ROS 2 workspace (the folder that contains src/). The Go2 CHAMP
# package is cloned and built in a SEPARATE workspace, ~/go2_ws, outside your repository;
# your workspace is built on top of it. The script also installs ROS 2 + Gazebo, creates
# the PyCharm virtualenv (~/venvs/go2) with the ROS 2 environment baked in, and installs
# the Unitree SDK. It never clones the private project repository and never writes
# third-party code into your repository.
#
# Run as your normal user, NOT with sudo (the script calls sudo where needed).
# Safe to re-run: every step skips work that is already done.
# Re-run it after 'colcon build' changes the workspace overlay, to refresh the venv.

set -eo pipefail

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------
GO2_CHAMP_REPOSITORY_URL="https://github.com/Humanoiderdepp/unitree_go2_ros2"
CHAMP_WORKSPACE_DIRECTORY="$HOME/go2_ws"
UNITREE_SDK_REPOSITORY_URL="https://github.com/unitreerobotics/unitree_sdk2_python.git"

SCRIPT_DIRECTORY="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# The workspace is the folder that contains src/: this script's folder, or its parent.
if [ -d "$SCRIPT_DIRECTORY/src" ]; then
  WORKSPACE_DIRECTORY="$SCRIPT_DIRECTORY"
elif [ -d "$(dirname "$SCRIPT_DIRECTORY")/src" ]; then
  WORKSPACE_DIRECTORY="$(dirname "$SCRIPT_DIRECTORY")"
else
  echo "ERROR: no src/ folder next to this script or one level above it." >&2
  echo "       Run the script from inside your clone of the project." >&2
  exit 1
fi
VENV_DIRECTORY="$HOME/venvs/go2"
UNITREE_SDK_DIRECTORY="$HOME/unitree_sdk2_python"
ROS_SETUP_FILE="/opt/ros/jazzy/setup.bash"
WORKSPACE_SETUP_FILE="$WORKSPACE_DIRECTORY/install/setup.bash"
VENV_BIN="$VENV_DIRECTORY/bin"

if [ -n "${SUDO_USER:-}" ] || [ "$(id -u)" -eq 0 ]; then
  echo "ERROR: run this script as your normal user, not with sudo." >&2
  echo "       (HOME is $HOME; the venv and workspace must live in your own home.)" >&2
  exit 1
fi

append_line_once() {
  # $1 = file, $2 = line. Avoids duplicate lines when the script is re-run.
  touch "$1"
  grep -qxF "$2" "$1" || echo "$2" >> "$1"
}

# ---------------------------------------------------------------------------
echo "### 1. Base system prep ###"
sudo apt update
sudo apt install -y locales software-properties-common curl gnupg lsb-release git python3-pip python3-venv
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

echo "### 2. ROS 2 apt repo and Jazzy ###"
sudo add-apt-repository universe -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install -y ros-jazzy-desktop ros-dev-tools

echo "### 3. Gazebo (OSRF apt repo) and Gazebo Harmonic ###"
sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt update
sudo apt install -y gz-harmonic

echo "### 4. ROS 2 <-> Gazebo bridge and project dependencies ###"
sudo apt install -y \
  ros-jazzy-ros-gz \
  ros-jazzy-ros2-control \
  ros-jazzy-ros2-controllers \
  ros-jazzy-xacro \
  ros-jazzy-robot-localization \
  ros-jazzy-velodyne \
  ros-jazzy-velodyne-description \
  ros-jazzy-nav2-bringup \
  ros-jazzy-slam-toolbox \
  ros-jazzy-pointcloud-to-laserscan \
  ros-jazzy-rtabmap-ros

echo "### 5. Source ROS 2 for this and future shells ###"
append_line_once "$HOME/.bashrc" "source $ROS_SETUP_FILE"
source "$ROS_SETUP_FILE"

echo "### 6. rosdep ###"
sudo apt install -y python3-rosdep
sudo rosdep init 2>/dev/null || true
rosdep update

echo "### 7. Go2 CHAMP package: its own workspace, outside your repository ###"
mkdir -p "$CHAMP_WORKSPACE_DIRECTORY/src"
if [ ! -d "$CHAMP_WORKSPACE_DIRECTORY/src/unitree_go2_ros2" ]; then
  git clone "$GO2_CHAMP_REPOSITORY_URL" "$CHAMP_WORKSPACE_DIRECTORY/src/unitree_go2_ros2"
fi
cd "$CHAMP_WORKSPACE_DIRECTORY"
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source "$CHAMP_WORKSPACE_DIRECTORY/install/setup.bash"

echo "### 8. Your workspace, built on top of the CHAMP workspace ###"
cd "$WORKSPACE_DIRECTORY"
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
append_line_once "$HOME/.bashrc" "source $CHAMP_WORKSPACE_DIRECTORY/install/setup.bash"
append_line_once "$HOME/.bashrc" "source $WORKSPACE_SETUP_FILE"
source "$WORKSPACE_SETUP_FILE"

echo "### 9. PyCharm virtualenv carrying the ROS 2 environment ###"
# PyCharm never sources setup.bash, so the venv's interpreter is a small wrapper
# that loads the ROS 2 environment and then hands off to the real interpreter,
# which is kept under its own name (.python-base). The wrapper must never exec a
# name that points back at itself, or it recurses until "Argument list too long".

if [ -z "${PYTHONPATH:-}" ]; then
  echo "ERROR: PYTHONPATH is empty after sourcing ROS. Did the workspace build fail?" >&2
  exit 1
fi

# Remove broken links and wrappers left by an earlier run, so venv recreates them.
for interpreter_name in python python3 python3.12; do
  link_path="$VENV_BIN/$interpreter_name"
  if [ -L "$link_path" ] && [ ! -e "$link_path" ]; then
    rm -f "$link_path"
  elif [ -f "$link_path" ] && grep -qE 'Generated by setup(-venv)?\.sh' "$link_path" 2>/dev/null; then
    rm -f "$link_path"
  fi
done

# --system-site-packages exposes apt-installed libraries (numpy etc.), not ROS.
python3 -m venv --system-site-packages "$VENV_DIRECTORY"

# pyvenv.cfg records the base interpreter, whatever state bin/ is in.
BASE_INTERPRETER="$(sed -n 's/^executable = //p' "$VENV_DIRECTORY/pyvenv.cfg" 2>/dev/null | head -1)"
if [ ! -x "$BASE_INTERPRETER" ]; then
  BASE_INTERPRETER="$(readlink -f "$(command -v python3 || echo /usr/bin/python3)")"
fi
if [ ! -x "$BASE_INTERPRETER" ]; then
  echo "ERROR: cannot find the interpreter this venv was built from." >&2
  echo "       Delete $VENV_DIRECTORY and re-run this script." >&2
  exit 1
fi
rm -f "$VENV_BIN/.python-base"
ln -s "$BASE_INTERPRETER" "$VENV_BIN/.python-base"

SITE_PACKAGES_DIRECTORY="$("$VENV_BIN/python" -c 'import site; print(site.getsitepackages()[0])')"

# Static ROS import paths, so importing works even with a bare environment.
printf '%s\n' "$PYTHONPATH" | tr ':' '\n' > "$SITE_PACKAGES_DIRECTORY/ros2.pth"

# The system module 'sip' (python3-sip 4.19.25) imports on Python 3.12 but segfaults
# at interpreter exit. PyCharm's skeleton generator imports it on purpose, so every
# run died with "failed to run generator3/__main__.py ... exit code 139". A stub in
# the venv shadows it. Do NOT 'apt remove python3-sip': it also removes matplotlib.
cat > "$SITE_PACKAGES_DIRECTORY/sip.py" <<'STUB'
# Installed by setup.sh: shadows the system sip, which segfaults at interpreter exit.
raise ImportError("sip is disabled for this venv (python3-sip 4.19.25 segfaults on Python 3.12)")
STUB

mkdir -p "$VENV_DIRECTORY/etc"
{
  echo "PYTHONPATH=$PYTHONPATH"
  echo "LD_LIBRARY_PATH=$LD_LIBRARY_PATH"
  echo "AMENT_PREFIX_PATH=${AMENT_PREFIX_PATH:-}"
} > "$VENV_DIRECTORY/etc/ros.env"

write_wrapper() {
  # rm first: these names may be symlinks, and 'cat > symlink' would write through them.
  rm -f "$1"
  cat > "$1" <<'EOF'
#!/bin/bash
# Generated by setup.sh - re-run that script after the workspace overlay changes.
# Shell builtins only: PyCharm starts interpreters with a minimal environment.
case "$0" in
  */*) HERE="${0%/*}" ;;
  *)   HERE="." ;;
esac
HERE="$(cd "$HERE" && pwd)"
VENV="${HERE%/*}"
BASE="$HERE/.python-base"

if [ ! -x "$BASE" ]; then
  echo "venv wrapper: $BASE is missing - re-run setup.sh" >&2
  exit 127
fi
read -r -n 2 _MAGIC < "$BASE" 2>/dev/null || true
if [ "$_MAGIC" = "#!" ]; then
  echo "venv wrapper: $BASE is a shell script, not an interpreter - re-run setup.sh" >&2
  exit 127
fi

_OLD_PYTHONPATH="$PYTHONPATH"
_OLD_LD_LIBRARY_PATH="$LD_LIBRARY_PATH"
_OLD_AMENT_PREFIX_PATH="$AMENT_PREFIX_PATH"

if [ -r "$VENV/etc/ros.env" ]; then
  . "$VENV/etc/ros.env"
else
  echo "venv wrapper: $VENV/etc/ros.env is missing - re-run setup.sh" >&2
fi

export PYTHONPATH="$PYTHONPATH${_OLD_PYTHONPATH:+:$_OLD_PYTHONPATH}"
export LD_LIBRARY_PATH="$LD_LIBRARY_PATH${_OLD_LD_LIBRARY_PATH:+:$_OLD_LD_LIBRARY_PATH}"
export AMENT_PREFIX_PATH="$AMENT_PREFIX_PATH${_OLD_AMENT_PREFIX_PATH:+:$_OLD_AMENT_PREFIX_PATH}"

exec "$BASE" "$@"
EOF
  chmod +x "$1"
}

write_wrapper "$VENV_BIN/python"
write_wrapper "$VENV_BIN/python3"
write_wrapper "$VENV_BIN/python3.12"

if ! env -i HOME="$HOME" "$VENV_BIN/python" -c "import sys, rclpy; assert sys.prefix != sys.base_prefix"; then
  echo "ERROR: the venv wrapper does not work; restoring plain interpreter links." >&2
  for interpreter_name in python python3 python3.12; do
    rm -f "$VENV_BIN/$interpreter_name"
    ln -s "$BASE_INTERPRETER" "$VENV_BIN/$interpreter_name"
  done
  exit 1
fi

# ---------------------------------------------------------------------------
echo "### 10. Unitree SDK inside that virtualenv ###"
if [ ! -d "$UNITREE_SDK_DIRECTORY" ]; then
  git clone "$UNITREE_SDK_REPOSITORY_URL" "$UNITREE_SDK_DIRECTORY"
fi
# Keep NumPy below 2: ROS 2 Jazzy's compiled packages are built against NumPy 1.x.
"$VENV_BIN/python" -m pip install "numpy<2" "opencv-python<4.12"
# Regular (non-editable) install so PyCharm can resolve it. ROS is sourced above,
# which lets pip build cyclonedds against the CycloneDDS that ROS 2 ships.
"$VENV_BIN/python" -m pip install "$UNITREE_SDK_DIRECTORY"

echo "### 11. Verification with a clean environment ###"
env -i HOME="$HOME" "$VENV_BIN/python" -c \
  "import numpy, rclpy, unitree_sdk2py; print('OK - numpy', numpy.__version__)"

echo
echo "Done. Open a new terminal. In PyCharm select the interpreter DIRECTORY:"
echo "  $VENV_DIRECTORY"
echo "(Settings > Project > Python Interpreter > Add Interpreter > Existing.)"
