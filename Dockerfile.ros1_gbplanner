FROM unified_autonomy:ros1_base

# This file assumes you are logged in as a user. You will need to use sudo to install packages.

# Install additional ROS and system packages including Gazebo
RUN apt-get update && apt-get install -y \
    python3-catkin-tools \
    libgoogle-glog-dev \
    ros-noetic-joy \
    ros-noetic-twist-mux \
    ros-noetic-interactive-marker-twist-server \
    ros-noetic-octomap-msgs \
    ros-noetic-octomap-ros \
    gazebo11 \
    libgazebo11-dev \
    ros-noetic-gazebo-ros \
    ros-noetic-gazebo-ros-pkgs \
    ros-noetic-gazebo-ros-control \
    ros-noetic-rviz \
    ros-noetic-costmap-2d \
    libtool \
    automake \
    autoconf \
    ros-noetic-tf-conversions \
    ros-noetic-xacro \
    && rm -rf /var/lib/apt/lists/*