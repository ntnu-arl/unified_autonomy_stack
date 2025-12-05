# Deployment

The Unified Autonomy Stack is designed to be deployed using **Docker** and **Docker Compose**. This ensures a consistent environment across development, simulation, and hardware deployment, as well as keeps the dependencies bundled together without intefering with the rest of the packages. This further adds modularity to the stack as any componenet can be replaced or modified without affecting the rest of the stack.

## Organization of the packages

### Workspace Structure

The Unified Autonomy Stack is divided across a set of ROS and ROS 2 workspaces that are located in the `unified_autonomy_stack/workspaces` folder. Each workspace represents a separate component such as a sensor driver, SLAM software, etc. containing the core packages and dependencies. 

The packages are tracked using `git` through `vcstool`. Separate `.repos` and `exact.repos` files (hereafter referred to as 'repos file') are created for each workspace and are located in the `unified_autonomy_stack/repos` folder. These files track the location, remote url, and branch/git commit hash for each package in that workspace. The `.repos` file tracks the branch, whereas, the `exact.repos` file tracks the exact commit hash for the package.
All workspace and corresponding repos file names start with the prefix `ws_`. E.g., the workspace for the simulator is `unified_autonomy_stack/workspaces/ws_sim` and the packages in it are listed in `unified_autonomy_stack/repos/ws_sim.repos` and `unified_autonomy_stack/repos/ws_sim_exact.repos`.

### Robot Bringup Package

Often times, it is important to know the state of the entire stack during a mission which includes the parameter files, launch files, scripts, etc. Hence, a bringup package called `robot_bringup` is created and located in `unified_autonomy_stack/workspaces/robot_bringup`. When launching the stack, this package is symlinked inside the `src` directory of each repository. The package has the config file, launch files, and scripts needed for launching the stack. `robot_bringup` is structured such that it will be compiled as a ROS or ROS 2 package based on the detected ROS version inside the docker container for each workspace.

### Tracking packages through vcstool

There are two scripts provided in the `unified_autonomy_stack/scripts` folder to clone the packages from the repos files or to update the repos files when any local changes are made.

1. Cloning all packages at the state in the repos files: `./scripts/import_all_repos.sh`
2. Updating the repos files to track the current state of the packages: `./scripts/export_all_repos.sh`.

To check the status of all packages in the stack:

```bash
cd unified_autonomy_stack/workspaces
vcs status
```

## Docker based deployment - Structuring the dockerfiles

The system is launched through docker compose services located in the docker-compose.\<purpose\>.yml files. We have crated a `Makefile` to provide easy to use commands to use the docker compose functionalities. The available commands are:

```txt
help                                Show the help message
images                              Build all Docker images
build                               Build all services in parallel (messy output)
build-sequential                    Build all services sequentially (one-by-one and slower, but cleaner output)
launch DOCKER_COMPOSE_FILE=file     Launch all services in the docker compose file named 'file'
stop                                Stop all launched services
restart                             art all launched services
status-all                          All Services Status (all profiles)
logs                                show logs from all services
```

As the entire stack runs through a set of docker containers, separate dockerfiles are created for containers having specific dependencies.
There are three main dockerfiles from which other dockerfiles are created:

1. `Dockerfile.ros1_base`:
    - Base image: ros:noetic-perception
    - Description: Ubuntu 20.04 docker with standard ROS packages and supporting linux packages
2. `Dockerfile.ros2_base`:
    - Base image: ros:humble-perception
    - Description: Ubuntu 22.04 docker with standard ROS 2 packages and supporting linux packages
3. `Dockerfile.ros2_cuda`:
    - Base image:
        - Default: A custom image (created from `Dockerfile.cuda_pytorch`) containing cuda 11.8 with pytorch 2.1
        - A cuda and pytorch bundled image for the system specific architecture can be used (e.g. for jetson boards)
    - Description: Ubuntu 22.04 image with cuda 11.8, pytorch 2.1, and ros2 humble

### Building the docker images
To build the docker images from the dockerfiles, the command `make images` is used. All dockerfiles are located in the root folder of the `unified_autonomy_stack` repository.
The `docker-bake.hcl` file lists the target images.
The structure of a target is as follows:
All docker images will be created under the repository `unified_autonomy` defined by the snippet:

```txt
variable "REGISTRY" {
  default = "unified_autonomy"
}
```

The `"default"` group specifies the list of target images to be generated:

```txt
group "default" {
  targets = ["ros1_base", "ros2_base", "ros1_gbplanner", "ros2_sim", "cuda_pytorch", "ros2_cuda", "ros2_nmpc", "ros2_cbf", "ros1-bridge-builder", "ros2_ros1_bridge", "ros2_rl", "ros2_vlm"]
}
```

Each target definition has the following structure:

```txt
target "target_name" {		# Target name (to be added to the group "default" for the image to be built)
  context    = "."
  dockerfile = "Dockerfile.target_name"		# Dockerfile to be used for this image
  tags       = ["${REGISTRY}:target_name"]	# Tag of the generated image. We recommend the target name, dockerfile name, and the image take to have the same name
  contexts   = {
    "unified_autonomy:ros2_base" = "target:ros2_base"	# Base image used in the dockerfile (unified_autonomy:ros1_base, unified_autonomy:ros2_base, unified_autonomy:ros2_cuda, or any other if custom base image is created)
  }
  network = "host"
}
```

## Docker based deployment - Running the stack

All operations (compilation and launching) are done using the docker compose tool. There are two types of docker compose files used in the stack.

The file `.env` stores the environment variables that can be used in the docker compose files.

### Building the code

The `docker-compose.build.yml` file contains all the services to build each of the workspaces.

??? note "A set of templates for ease of use"
    ```yaml
    x-system-template: &system-template
    # runtime: nvidia
    working_dir: /workspace
    network_mode: host
    ipc: host
    restart: "no"
    stdin_open: true
    tty: true
    environment:
    - DISPLAY=${DISPLAY:-:0}
    - NVIDIA_VISIBLE_DEVICES=all
    - NVIDIA_DRIVER_CAPABILITIES=all
    - QT_X11_NO_MITSHM=1
    - DISABLE_ROS1_EOL_WARNINGS=1
    volumes:
        - /tmp/.X11-unix:/tmp/.X11-unix:rw

    x-x11-volume: &x11-volume
    /tmp/.X11-unix:/tmp/.X11-unix:rw

    x-robot-bringup-volume: &robot-bringup-volume
    ./workspaces/robot_bringup:/workspace/src/robot_bringup:rw

    x-recorded-data-volume: &recorded-data-volume
    ./data:/data:rw

    x-catkin-build-template: &catkin-build-template
    <<: *system-template
    command: catkin build -DCMAKE_BUILD_TYPE=Release
    profiles: ["build"]

    x-colcon-build-template: &colcon-build-template
    <<: *system-template
    command: colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
    profiles: ["build"]

    x-ros1-wait-for-roscore: &ros1-wait-for-roscore
    depends_on:
        ros1_launch_roscore:
        condition: service_healthy

    x-ros1-launch-template: &ros1-launch-template
    <<: [*system-template, *ros1-wait-for-roscore]
    profiles: ["launch"]

    x-ros2-launch-template: &ros2-launch-template
    <<: [*system-template]
    profiles: ["launch"]
    ```

The structure of a build service is as follows:
The build service can be a ROS (catkin) or ROS 2 (colcon) build service.
Two templates are created for each:

```yaml
x-catkin-build-template: &catkin-build-template
  <<: *system-template
  command: catkin build -DCMAKE_BUILD_TYPE=Release
  profiles: ["build"]

x-colcon-build-template: &colcon-build-template
  <<: *system-template
  command: colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release 
  profiles: ["build"]
```

A build service can use one of these (depending on the ros version). A typical build service looks as follows:

```yaml
build_<target>:
	<< [*catkin-build-template] # Use *colcon-build-template for ROS 2
	image: <docker image>
	command: <command> # The templates provide the default commands to build a catkin or colcon package. If a custom command is needed, then use this field otherwise don't add it.
	volumes:
		- *robot-bringup-volume # This mounts the robot_bringup package inside the /workspace/src folder
		- ./workspaces/ws_<name>:/workspace # Mount the workspace inside which this build command will run
		- ... # Any other volumes
	... # Other environment variables, or tags (e.g., runtime: nvidia for containers that need gpu access)
```

**Note:** The docker containers are NOT persistent. Hence, all the code and persistent files are in the workspaces which are then mounted to the directory `/workspace` inside the docker when the container is created. Hence, when the code compiles, the installed files (build, install, devel directories) remain outside the docker so the compiled binaries remain even when the container is stopped.

All the build services are located inside the `docker-compose.build.yml` file. When the `make build` or `make build-sequential` command is used, it runs all the services of the profile `"build"` in the `docker-compose.build.yml` file.

### Launching the stack

We provide a set of [examples](examples.md) to test the autonomy stack. We describe the structure of the docker compose files for launching the stack below.

Since each mission might not require running all the packages, a separate docker compose file should be created with the specific services for a particular mission. We provide several docker compose files for the demo scenarios which can be used for reference.

To launch all the services of the profile `"launch"` in a docker compose file, from the `unified_autonomy_stack` directory run:

```bash
make launch DOCKER_COMPOSE_FILE=filename
```

The structure of a docker compose file is as follows. It contains the same templates as in the `docker-compose.build.yml`

Among the services, a common service across all files having ROS related packages is the roscore:

```yaml
services:
  ros1_launch_roscore:
    <<: [*system-template]
    image: unified_autonomy:ros1_base
    profiles: ["launch"]
    command: roscore
    healthcheck:
      test: ["CMD-SHELL", "bash", "-c", "source /opt/ros/noetic/setup.bash && rostopic list || exit 1"]
      interval: 2s
      timeout: 2s
      retries: 5
      start_period: 3s  # Give it time to start before checking
```

This launches a roscore. The ros1-launch-template waits for the roscore service to start before launching any other ros1 service

??? note "Example ROS service"
    ```yaml
    ros1_launch_<target>:
        <<: [*ros1-launch-template]
        image: <docker image>
        command: <command> # It is prefered to add the launch files to the robot_bringup package and lauch them here so that the state of the entire stack can be tracked through that one package.
        volumes:
        - *x11-volume # Optional
        - *robot-bringup-volume # This mounts the robot_bringup package inside the /workspace/src folder
                - ./workspaces/ws_<name>:/workspace # Mount the workspace inside which this build command will run
                - ... # Any other volumes
            ... # Other environment variables, or tags (e.g., runtime: nvidia for containers that need gpu access)
    ```

??? note "Example ROS 2 service"
    ```yaml
    ros2_launch_<target>:
        <<: [*ros2-launch-template]
        image: <docker image>
        command: <command> # It is prefered to add the launch files to the robot_bringup package and lauch them here so that the state of the entire stack can be tracked through that one package.
        volumes:
        - *x11-volume # Optional
        - *robot-bringup-volume # This mounts the robot_bringup package inside the /workspace/src folder
                - ./workspaces/ws_<name>:/workspace # Mount the workspace inside which this build command will run
                - ... # Any other volumes
            environment:
        - ROS_DOMAIN_ID=${DOMAIN_ID} # The parameter DOMAIN_ID is set in the .env file
                - ... # OTher environment variables
            ... # Other environment variables, or tags (e.g., runtime: nvidia for containers that need gpu access)
    ```
