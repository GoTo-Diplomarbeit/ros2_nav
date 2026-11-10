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
    pip install --break-system-packages --ignore-installed -r requirements.txt

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
# Switch to the workspace folder
WORKDIR /unitree_go2_sim
# Changes whenever a new commit is pushed (invalidates the cache from here on)
ADD https://api.github.com/repos/GoTo-Diplomarbeit/unitree_go2_ros2/git/refs/heads/main /tmp/repo_version

# Clone the repository into the unitree_go2_sim folder of its own workspace
RUN git clone --recursive https://github.com/GoTo-Diplomarbeit/unitree_go2_ros2 .
# Install all required packages
RUN rosdep install --from-paths . --ignore-src -r -y
# Build the unitree_go2_ros2 project
RUN . /opt/ros/jazzy/setup.sh && colcon build

# -- Add the ROS python folders to Python's search path via a .pth file (so PyCharm finds rclpy etc.)
RUN printf "/opt/ros/jazzy/lib/python3.12/site-packages\n/opt/ros/jazzy/local/lib/python3.12/dist-packages\n" > /usr/local/lib/python3.12/dist-packages/ros.pth

# -- Add env variables for packages which are installed via ros2 rather than pip
ENV AMENT_PREFIX_PATH=/opt/ros/jazzy
ENV PYTHONPATH=/opt/ros/jazzy/lib/python3.12/site-packages:/opt/ros/jazzy/local/lib/python3.12/dist-packages
ENV LD_LIBRARY_PATH=/opt/ros/jazzy/lib
ENV ROS_DISTRO=jazzy

# ------------------
# | Finished setup |
# ------------------

# -- Source ROS, the unitree workspace and the goto project automatically in every interactive terminal
# Source ROS2
RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc
# Source the simulator
RUN echo "source /unitree_go2_sim/install/setup.bash" >> /root/.bashrc
# Source our project
RUN echo "source /ws/install/setup.bash 2>/dev/null || true" >> /root/.bashrc

# -- Work around a Python 3.12 crash in Ubuntu's sip4 package
# PyCharm's skeleton generator imports the legacy top-level `sip` module. Ubuntu's
# python3-sip 4.19.25 registers a Py_AtExit handler that calls PyMem_Free during
# interpreter finalization; Python 3.12 has already finalized the GIL at that point,
# so the process dies with SIGSEGV (exit code 139). PyCharm then reports "Couldn't
# refresh skeletons" and leaves every import unresolved.
# python3-sip cannot be removed because ROS rqt/rtabmap depend on it, so shadow it
# with PyQt5's maintained sip implementation (see the shim for details).
COPY scripts/fix/sip_shim.py /usr/local/lib/python3.12/dist-packages/sip.py

# -- Open a terminal when executing docker (can be overridden by other commands)
# Open terminals in the project folder
WORKDIR /ws

# Run commands as bash (can be overriden eg. for using python)
CMD ["bash"]
