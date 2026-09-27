# How to install this project
## First install prerequisites
### First install Ubuntu 24.04 LTS either on a laptop or on wsl2
#### For WSL:
```bash
wsl --install -d Ubuntu-24.04
```
### Then run these commands
```bash
### 1. Base system prep ###
sudo apt update && sudo apt upgrade -y
sudo apt install -y locales software-properties-common curl gnupg lsb-release git python3-pip python3-venv
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

### 2. Add ROS 2 apt repo and install Jazzy ###
sudo add-apt-repository universe -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install -y ros-jazzy-desktop ros-dev-tools

### 3. Add Gazebo (OSRF) apt repo and install Gazebo Harmonic ###
sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt update
sudo apt install -y gz-harmonic

### 4. ROS2<->Gazebo bridge + repo-specific dependencies ###
sudo apt install -y \
  ros-jazzy-ros-gz \
  ros-jazzy-ros2-control \
  ros-jazzy-ros2-controllers \
  ros-jazzy-xacro \
  ros-jazzy-robot-localization \
  ros-jazzy-velodyne \
  ros-jazzy-velodyne-description

### 5. Source ROS2 for this and future shells ###
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source /opt/ros/jazzy/setup.bash

### 6. rosdep ###
sudo apt install -y python3-rosdep
sudo rosdep init 2>/dev/null || true
rosdep update

### 7. Workspace + clone the Go2 CHAMP package ###
mkdir -p ~/go2_ws/src
cd ~/go2_ws/src
git clone https://github.com/Humanoiderdepp/unitree_go2_ros2

### 8. Resolve remaining deps and build ###
cd ~/go2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
echo "source ~/go2_ws/install/setup.bash" >> ~/.bashrc
source ~/go2_ws/install/setup.bash
```
