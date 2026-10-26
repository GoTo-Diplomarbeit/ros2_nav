# -- Install ubuntu and ros:jazzy
FROM ros:jazzy AS dev
LABEL authors="Peter Promok"

# -- Refresh package list
RUN apt-get update

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
# Install the required python packages using break-system-packages
RUN pip install --break-system-packages -r requirements.txt

# ------------------
# | Finished setup |
# ------------------

# -- Open a terminal when executing docker
ENTRYPOINT ["bash"]
