FROM ros:noetic-perception

# Prevent interactive prompts during package installation
ENV DEBIAN_FRONTEND=noninteractive

# Install additional development tools and GUI support
RUN apt-get update && apt-get install -y \
    git \
    vim \
    nano \
    htop \
    curl \
    wget \
    sudo \
    python3-pip \
    python3-catkin-tools \
    python3-osrf-pycommon \
    libgoogle-glog-dev \
    libspdlog-dev \
    libpcap-dev \
    x11-apps \
    mesa-utils \
    libgl1-mesa-glx \
    libgl1-mesa-dri \
    bash-completion \
    apt-file \
    net-tools \
    iputils-ping \
    iputils-tracepath \
    ssh \
    build-essential \
    cmake \
    gdb \
    valgrind \
    clang-format \
    tmux \
    python3-dev \
    python3-setuptools \
    python3-wheel \
    tree \
    less \
    unzip \
    software-properties-common \
    v4l-utils \
    && rm -rf /var/lib/apt/lists/* \
    && apt-file update

# Dependancies for mavlink and mavros. This should be removed once mavros version 1.22 is available from apt https://github.com/mavlink/mavros/milestone/68 at which point we dont need to build mavros and mavlink from source anymore.
RUN apt-get update && apt-get install -y \
    libgeographic-dev \
    geographiclib-tools \
    ros-noetic-geographic-msgs \
    ros-noetic-diagnostic-updater \
    ros-noetic-urdf \
    ros-noetic-control-toolbox \
    ros-noetic-tf2-sensor-msgs \
    ros-noetic-mavros-msgs \
    ros-noetic-tf-conversions \
    ros-noetic-rviz \
    && rm -rf /var/lib/apt/lists/* \
    && apt-file update
RUN pip3 install future

# Initialize rosdep for the user (rosdep init was already run in the base image)
RUN rosdep update

# Enable bash completion
RUN echo "if [ -f /etc/bash_completion ]; then . /etc/bash_completion; fi" >> ~/.bashrc

COPY ./entrypoint.sh /
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
