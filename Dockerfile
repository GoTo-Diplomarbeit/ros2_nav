# -- Install ubuntu and ros:jazzy
FROM ros:jazzy AS dev
LABEL authors="Peter Promok"

# -- Refresh package list
RUN apt-get update && rosdep update

# ----------------------------------------
# | Install the required python packages |
# ----------------------------------------

# -- Install git, pip, cmake and build-essential
RUN apt-get install -y git python3-pip cmake build-essential

# -- Build CycloneDDS (needed by unitree_sdk2py)
# Clone cyclonedds into /cyclonedds
RUN git clone https://github.com/eclipse-cyclonedds/cyclonedds -b releases/0.10.x /cyclonedds
# Create a workdirectory for the build folder
WORKDIR /cyclonedds/build
# Run cmake to configure the CycloneDDS build to use the build and install folder
RUN cmake .. -DCMAKE_INSTALL_PREFIX=../install
# Build CycloneDDS via cmake into the install folder
RUN cmake --build . --target install
# Set the cyclonedds_home env variable
ENV CYCLONEDDS_HOME=/cyclonedds/install

# -- Installs the python packages
# Creates a directory for the workspace
WORKDIR /ws
# Copy requirements.txt to the container workspace
COPY requirements.txt .
# Install the required python packages, with a persistent pip cache
RUN --mount=type=cache,target=/root/.cache/pip \
    pip install --break-system-packages -r requirements.txt

# -- Add the ROS python folders to Python's search path via a .pth file (so PyCharm finds rclpy etc.)
RUN printf "/opt/ros/jazzy/lib/python3.12/site-packages\n/opt/ros/jazzy/local/lib/python3.12/dist-packages\n" > /usr/local/lib/python3.12/dist-packages/ros.pth

# -- Add env variables for packages which are installed via ros2 rather than pip
ENV AMENT_PREFIX_PATH=/opt/ros/jazzy
ENV PYTHONPATH=/opt/ros/jazzy/lib/python3.12/site-packages:/opt/ros/jazzy/local/lib/python3.12/dist-packages
ENV LD_LIBRARY_PATH=/opt/ros/jazzy/lib
ENV ROS_DISTRO=jazzy

# --------------------------------------
# | Install the required ros2 packages |
# --------------------------------------
# Switch to the ws directory
WORKDIR /ws
# Copy the package.xml file to docker
COPY /src/goto/package.xml .
# Install every package declared inside the file
RUN rosdep install --from-paths . --ignore-src -r -y

# ---------------------------------------
# | Clone and build the unitree_go2_sim |
# ---------------------------------------
# Install Gazebo Harmonic and the ROS-Gazebo bridge
RUN apt-get install -y ros-jazzy-ros-gz ros-jazzy-ros2controlcli ros-jazzy-ros2-controllers
# Clone the repository into the src folder of its own workspace
RUN git clone --recursive https://github.com/GoTo-Diplomarbeit/unitree_go2_ros2 /unitree_ws/src/unitree_go2_ros2
# Switch to the workspace folder
WORKDIR /unitree_ws
# Install all required packages
RUN rosdep install --from-paths src --ignore-src -r -y
# Build the unitree_go2_ros2 project
RUN . /opt/ros/jazzy/setup.sh && colcon build

# ------------------
# | Finished setup |
# ------------------

# Open terminals in the project folder
WORKDIR /ws

# -- Source ROS, the unitree workspace and your own project automatically in every interactive terminal
RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc
RUN echo "source /unitree_ws/install/setup.bash" >> /root/.bashrc
RUN echo "source /ws/install/setup.bash 2>/dev/null || true" >> /root/.bashrc

# -- Open a terminal when executing docker (can be overridden by other commands)
CMD ["bash"]
