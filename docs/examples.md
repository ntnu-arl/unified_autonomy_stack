# Examples

We have created a set of examples that the user can run to test the stack and get familiarized with it.

Each example has it's own docker compose file that contains all the services necessary to run the example.

## Simulation - UAV Exploration

Upon launching the simulation, follow the steps below to start the mission

- Click the `Initialize` button in the UI. Wait until you see the message `initMotion() - Done` in the terminal.
- Click the `Start Planner` button to start the mission.

More details about the UI can be found [here](https://github.com/ntnu-arl/gbplanner_ros/wiki/Demo#description-of-the-controls-provided-by-the-ui).

![UI](./figures/ui.png)

### Using NMPC

#### Cylindrical lidar

```bash
cd unified_autonomy_stack
make launch DOCKER_COMPOSE_FILE=docker-compose.uav_nmpc_sim.yml
```

#### Dome lidar

```bash
cd unified_autonomy_stack
make launch DOCKER_COMPOSE_FILE=docker-compose.uav_nmpc_unipilot_sim.yml
```

### Using RL

#### Dome lidar

```bash
cd unified_autonomy_stack
make launch DOCKER_COMPOSE_FILE=docker-compose.uav_rl_sim.yml
```

## Simulation - UGV Exploration

```bash
cd unified_autonomy_stack
make launch DOCKER_COMPOSE_FILE=docker-compose.ugv_sim.yml 
```

## SLAM Example


Download the dataset to test the SLAM from [here](https://huggingface.co/datasets/ntnu-arl/unified_autonomy_stack_datasets/tree/main/l%C3%B8kken_mine)

**Terminal 1 (All necessary ROS packages)**:

```bash
cd unified_autonomy_stack
make launch DOCKER_COMPOSE_FILE=docker-compose.slam_demo.yml
```

**Terminal 2 (Playing the rosbag)**:

If you have ROS installed locally on your system, you can play the rosbag directly on your machine:

```bash
rosbag play --clock <bag_name>.bag
```

If you do not have ROS installed locally, we provide a docker service to run the rosbags:

```bash
cd unified_autonomy_stack
BAG_FOLDER=<path to the directory where the bag is stored> BAG_NAME=<bag_name>.bag  docker compose -f docker-compose.rosbag_play.yml --profile launch up
```
