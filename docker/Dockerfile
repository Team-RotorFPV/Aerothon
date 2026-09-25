# AeroTHON 2026 Mission 2 — dev/sim container
# ROS 2 Jazzy + Gazebo Harmonic + SITL toolchain on Ubuntu 24.04.
FROM ros:jazzy-ros-base-noble

SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

# ROS packages + Gazebo Harmonic bridge + tooling
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-jazzy-ros-gz \
      ros-jazzy-mavros ros-jazzy-mavros-extras \
      ros-jazzy-slam-toolbox \
      ros-jazzy-navigation2 ros-jazzy-nav2-costmap-2d \
      ros-jazzy-web-video-server \
      python3-vcstool python3-colcon-common-extensions python3-pip \
      git wget curl gz-harmonic \
    && rm -rf /var/lib/apt/lists/*

# MAVROS GeographicLib datasets (required for global<->local)
RUN wget -q https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh \
    && bash install_geographiclib_datasets.sh || true

# Python deps for perception / mission (CPU OpenCV, py_trees)
RUN pip3 install --break-system-packages \
      py_trees py_trees_ros opencv-python numpy pyproj

WORKDIR /aerothon_ws
# Source ROS in every shell
RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc \
 && echo "[ -f /aerothon_ws/install/setup.bash ] && source /aerothon_ws/install/setup.bash" >> /root/.bashrc

CMD ["bash"]
