
# Deep Reinforcement Learning (DRL)

The Deep Reinforcement Learning (DRL) navigation module employs policies that utilize both the robot odometry and instantaneous raw exteroceptive depth information from depth cameras or LiDAR sensors. The method exploits local ego-centric representations of the goal location and commands reference body velocity or acceleration (WIP) setpoints. The sensor can be compressed into a compact latent representation using a neural encoder. Alternatively, raw images can be downsampled and processed with convolutional layers. Policies are trained in simulation using the [Aerial Gym Simulator](https://github.com/ntnu-arl/aerial_gym_simulator). 

This method enables safe collision-free navigation in complex environments without a map, and thus provides an add-on safety layer to traditional map-based planning approaches, offering resilience when the onboard SLAM experiences drift or when certain obstacles are not mapped sufficiently.

!!! note "Source Code"
    - **Workspace:** `workspaces/ws_rl/src`
    - **Package:** `rl_nav`
    - **GitHub:** [ntnu-arl/rl_nav](https://github.com/ntnu-arl/rl_nav/tree/dev/unified_autonomy_stack)

**Related Publications**:

- [M. Kulkarni, W. Rehberg and K. Alexis, "Aerial Gym Simulator: A Framework for Highly Parallelized Simulation of Aerial Robots," in IEEE Robotics and Automation Letters, vol. 10, no. 4, pp. 4093-4100, April 2025.](https://ieeexplore.ieee.org/document/10910148)
- [Kulkarni, Mihir, and Kostas Alexis. "Reinforcement learning for collision-free flight exploiting deep collision encoding." 2024 IEEE International Conference on Robotics and Automation (ICRA). IEEE, 2024.](https://ieeexplore.ieee.org/stamp/stamp.jsp?arnumber=10610287)

<!-- Create an admonition or note for markdown -->
??? note "Aerial Gym Simulator"
    The [Aerial Gym Simulator](https://ntnu-arl.github.io/aerial_gym_simulator/) is used to train and evaluate the RL navigation policy. The training environments and policies used for this task are [open-sourced](https://github.com/ntnu-arl/aerial_gym_simulator).

## Observations

- $[\mathbf{e}_p, \phi, \theta, e_{\psi}, \mathbf{v}, \boldsymbol{\omega}, \mathbf{a}_{t-1}, \mathbf{C}]$
    - $\mathbf{e}_p$ is the position error to the goal in the body frame.
    - $\phi, \theta$ are roll and pitch angles.
    - $e_{\psi}$ is the yaw error to the goal direction.
    - $\mathbf{v}, \boldsymbol{\omega}$ are linear and angular velocities in the body frame.
    - $\mathbf{a}_{t-1}$ is the previous action taken by the agent.
- $\mathbf{C}$ is a representation of the sensor data
    - $\mathbf{C}$ is the downsampled LiDAR range image (16 × 20 grid) in the case for LiDAR-based navigation.
    - $\mathbf{C}$ is the compressed latent representation encoding collision information when using a VAE-based encoder and a depth camera.

where $\mathbf{e}_p$ is the position error to the goal in the body frame, $\phi, \theta$ are roll and pitch angles, $\mathbf{e}_{\psi}$ is the yaw error to the goal direction, $\mathbf{v}, \boldsymbol{\omega}$ are linear and angular velocities in the body frame, and $\mathbf{a}_{t-1}$ is the previous action taken by the agent.

### Actions and Bounds

The policy outputs a normalized action vector $\mathbf{a}_t \in [-1,1]^4$, which is mapped to body-frame velocity commands $[v_x, v_y, v_z, \dot{\psi}] \in \mathbb{R}^4$:

$$v_x = -(a_0 + 1), \quad v_y = a_1, \quad v_z = a_2, \quad \dot{\psi} = a_3 \cdot \tfrac{\pi}{3}$$

This mapping produces velocities $v_x \in [-2, 0]$ m/s (The LiDAR is mounted backwards), lateral/vertical velocities $v_y, v_z \in [-1, 1]$ m/s, and yaw rate $\dot{\psi} \in [-\frac{\pi}{3}, \frac{\pi}{3}]$ rad/s.



### Reward Function

The reward function encourages goal progress, directional alignment, stability, and safety through shaped rewards and penalties. All rewards are combined and scaled by curriculum progress.

**Helper Functions:**

- Exponential reward: $\mathcal{R}(m, e, x) = m \cdot \exp(-e \cdot x^2)$
- Exponential penalty: $\mathcal{P}(m, e, x) = m \cdot (\exp(-e \cdot x^2) - 1)$

??? note "Detailed Reward Components"
    | Category                | Term                      | Formula                                                                                                 | Description                                        |
    | ----------------------- | ------------------------- | ------------------------------------------------------------------------------------------------------- | -------------------------------------------------- |
    | **Attraction to Goal**  | $R_{\text{pos}}$          | $\mathcal{R}(3, 1, d)$                                                                                  | Exponential reward based on distance to goal       |
    |                         | $R_{\text{close}}$        | $\mathcal{R}(5, 8, d) \cdot \alpha_{\psi}$                                                              | Bonus when very close, gated by yaw alignment      |
    |                         | $R_{\text{dist}}$         | $\frac{20-d}{20}$                                                                                       | Linear reward for reducing distance                |
    | **Direction Alignment** | $R_{\text{dir}}$          | $\hat{\mathbf{v}} \cdot \hat{\mathbf{p}}_{\text{goal}} \cdot R_{\text{vel}} \cdot \min(\frac{d}{3}, 1)$ | Velocity direction toward goal, scaled by distance |
    |                         | $R_{\text{vel}}$          | $\mathcal{R}(2, 2, \|\mathbf{v}\|-2)$                                                                   | Reward for desired velocity (~2 m/s)               |
    |                         | $\alpha_{\psi}$           | $\mathcal{R}(1, 2, e_{\psi})$                                                                           | Yaw alignment factor                               |
    | **Stability at Goal**   | $R_{\text{stable}}$       | $R_{\text{low_vel}} + R_{\psi} + R_{\text{low}_\omega}$                                                 | Active only when $d < 1$ m                         |
    |                         | $R_{\text{low_vel}}$      | $\mathcal{R}(1.5, 10, \|\mathbf{v}\|) + \mathcal{R}(1.5, 0.5, \|\mathbf{v}\|)$                          | Low velocity reward                                |
    |                         | $R_{\psi}$                | $\mathcal{R}(2, 0.2, e_{\psi}) + \mathcal{R}(4, 15, e_{\psi})$                                          | Correct yaw orientation                            |
    |                         | $R_{\text{low}_\omega}$   | $\mathcal{R}(1.5, 5, \omega_z) \cdot \alpha_{\psi}$                                                     | Low yaw rate, gated by alignment                   |
    | **Velocity Penalties**  | $P_{\text{vel}}$          | $\mathcal{P}(2, 2, \|\mathbf{v}\|-3)$                                                                   | Penalize speeds $> 3$ m/s                          |
    |                         | $P_{\text{bwd}}$          | $\mathcal{P}(2, 8, v_x)$                                                                                | Penalize positive $v_x$ (backward motion)          |
    | **Action Smoothness**   | $P_{\text{diff},i}$       | $\mathcal{P}(0.1, 5, a_{i,t} - a_{i,t-1})$                                                              | Smooth action transitions                          |
    | **Absolute Action**     | $P_{\text{abs},x/y}$      | $C \cdot \mathcal{P}(0.1, 0.3, a_i)$                                                                    | Penalize large $x$/$y$ actions                     |
    |                         | $P_{\text{abs},z}$        | $C \cdot \mathcal{P}(0.15, 1.0, a_z)$                                                                   | Penalize large $z$ action                          |
    |                         | $P_{\text{abs},\dot\psi}$ | $C \cdot \mathcal{P}(0.15, 2.0, a_{\dot\psi})$                                                          | Penalize large yaw rate action                     |
    | **Safety**              | $P_{\text{TTC}}$          | $-\mathcal{R}(3, 1, \text{TTC})$                                                                        | Time-to-collision penalty                          |
    |                         | $P_{\text{crash}}$        | $-10$                                                                                                   | Collision penalty                                  |

    where $d$ is distance to goal, $e_{\psi}$ is yaw error.

**Time-to-Collision (TTC):**

For each LiDAR ray with unit direction $\hat{\mathbf{u}}_{\text{ray}}$, range $r$, and robot velocity $\mathbf{v}$:

$$\text{TTC}_{\text{ray}} = \begin{cases} \frac{r}{\mathbf{v} \cdot \hat{\mathbf{u}}_{\text{ray}}} & \text{if approaching} \\ +\infty & \text{otherwise} \end{cases}$$

The TTC metric is the minimum across all rays, encouraging the agent to maintain safe distance.

**Total Reward:**

$$R_{\text{total}} = \gamma \left( R_{\text{pos}} + R_{\text{close}} + R_{\text{dir}} + R_{\text{dist}} + R_{\text{stable}} + P_{\text{vel}} + P_{\text{action}} + P_{\text{TTC}} \right)$$

where $\gamma = 1 + 2 C_{\text{prog}}$ scales rewards based on curriculum progress $C_{\text{prog}} \in [0,1]$.

## RL Topics & Interfaces

### Input

| Topic                | Type                        | Description                                      |
| -------------------- | --------------------------- | ------------------------------------------------ |
| `/msf_core/odometry` | `nav_msgs/Odometry`         | State estimate from MSF                          |
| `/target`            | `geometry_msgs/PoseStamped` | Target position from planner or user             |
| `/gbplanner_path`    | `nav_msgs/Path`             | Path from planner; last pose extracted as target |
| `/rslidar_points`    | `sensor_msgs/PointCloud2`   | LiDAR point cloud                                |
| `/reset`             | `std_msgs/Empty`            | Reset policy hidden state and action filter      |
| `/rmf/mavros/state`  | `mavros_msgs/State`         | MAVROS state (optional, for OFFBOARD check)      |

### Output

| Topic                        | Type                         | Description                              |
| ---------------------------- | ---------------------------- | ---------------------------------------- |
| `/cmd_vel`                   | `geometry_msgs/Twist`        | Velocity setpoint for trajectory tracker |
| `/cmd_vel_filtered`          | `geometry_msgs/Twist`        | EMA-filtered velocity command            |
| `/mavros/setpoint_raw/local` | `mavros_msgs/PositionTarget` | Direct velocity command to PX4           |
| `/cmd_vel_viz`               | `geometry_msgs/TwistStamped` | Velocity command (visualization)         |

## RL Configuration

Configuration defined in the `Config` class within `lidar_nav_vel_ROS 2_node.py`.

### Observation Space

| Parameter       | Description                                                                                                          |
| --------------- | -------------------------------------------------------------------------------------------------------------------- |
| `STATE_DIM`     | State observation dimension (17): $[e_p, \phi, \theta, e_{\psi}, \mathbf{v}, \boldsymbol{\omega}, \mathbf{a}_{t-1}]$ |
| `LIDAR_DIM`     | Downsampled LiDAR grid size (16 × 20 = 320)                                                                          |
| `TOTAL_OBS_DIM` | Total observation dimension (337)                                                                                    |

### LiDAR Processing

| Parameter                   | Description                                      |
| --------------------------- | ------------------------------------------------ |
| `IMAGE_HEIGHT`              | Binned LiDAR height (48)                         |
| `IMAGE_WIDTH`               | Binned LiDAR width (480)                         |
| `LIDAR_MAX_RANGE`           | Maximum range clipping (m, typically 10.0)       |
| `LIDAR_MIN_RANGE`           | Minimum range clipping (m, typically 0.4)        |
| `MEDIAN_FILTER`             | Enable median filtering for noise removal (bool) |
| `MEDIAN_FILTER_KERNEL_SIZE` | Median filter kernel size (typically 7)          |

### Action Space

| Parameter      | Description                                         |
| -------------- | --------------------------------------------------- |
| `ACTION_DIM`   | Action dimension (4: vx, vy, vz, yaw_rate)          |
| `ACTION_SCALE` | Scaling factors for actions `[1.0, 1.0, 0.75, 1.0]` |

### Control

| Parameter             | Description                          |
| --------------------- | ------------------------------------ |
| `ACTION_FILTER_ALPHA` | EMA filter coefficient (0.3)         |
| `USE_MAVROS_STATE`    | Enable MAVROS state checking (bool)  |
| `DEVICE`              | Inference device (`cuda:0` or `cpu`) |

### Frame IDs

| Parameter       | Description                                  |
| --------------- | -------------------------------------------- |
| `BODY_FRAME_ID` | Body frame for visualization (`mimosa_body`) |

---

## Citation

If you use the RL navigation module in your work, please cite the relevant publication:

```bibtex
@inproceedings{navigation_drl,
	title = {Reinforcement {Learning} for {Collision}-free {Flight} {Exploiting} {Deep} {Collision} {Encoding}},
	url = {https://ieeexplore.ieee.org/document/10610287},
	doi = {10.1109/ICRA57147.2024.10610287},
	booktitle = {2024 {IEEE} {International} {Conference} on {Robotics} and {Automation} ({ICRA})},
	author = {Kulkarni, Mihir and Alexis, Kostas},
	month = may,
	year = {2024},
}

@article{navigation_aerialgym,
	title = {Aerial {Gym} {Simulator}: {A} {Framework} for {Highly} {Parallelized} {Simulation} of {Aerial} {Robots}},
	volume = {10},
	issn = {2377-3766},
	shorttitle = {Aerial {Gym} {Simulator}},
	url = {https://ieeexplore.ieee.org/document/10910148/},
	doi = {10.1109/LRA.2025.3548507},
	number = {4},
	journal = {IEEE Robotics and Automation Letters},
	author = {Kulkarni, Mihir and Rehberg, Welf and Alexis, Kostas},
	month = apr,
	year = {2025},
}

@inproceedings{navigation_dce,
	address = {Cham},
	title = {Task-{Driven} {Compression} for {Collision} {Encoding} {Based} on {Depth} {Images}},
	isbn = {978-3-031-47966-3},
	doi = {10.1007/978-3-031-47966-3_20},
	booktitle = {Advances in {Visual} {Computing}},
	publisher = {Springer Nature Switzerland},
	author = {Kulkarni, Mihir and Alexis, Kostas},
	editor = {Bebis, George and Ghiasi, Golnaz and Fang, Yi and Sharf, Andrei and Dong, Yue and Weaver, Chris and Leo, Zhicheng and LaViola Jr., Joseph J. and Kohli, Luv},
	year = {2023},
	pages = {259--273},
}
```