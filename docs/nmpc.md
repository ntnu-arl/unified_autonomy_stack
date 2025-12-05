# Neural SDF-MPC (NMPC)

Neural SDF-MPC is a nonlinear model predictive control framework for mapless, collision-free navigation in unknown environments using onboard range sensing. The method leverages deep neural networks to encode a single range image—capturing all available information about the environment—into a Signed Distance Function (SDF). This representation is used to formulate collision avoidance constraints within a receding-horizon optimal control problem, solved using an efficient SQP-RTI scheme.

!!! note "Source Code"
    - **Workspace:** `workspaces/ws_nmpc/src`
    - **Package:** `sdf_nmpc_ros`
    - **GitHub:** [ntnu-arl/sdf_nmpc_ros](https://github.com/ntnu-arl/sdf_nmpc_ros/tree/release/unified_autonomy_stack)

**Related Publication**:

- [Jacquet, Martin, Marvin Harms, and Kostas Alexis. "Neural NMPC through Signed Distance Field Encoding for Collision Avoidance." arXiv preprint arXiv:2511.21312 (2025).](https://arxiv.org/abs/2511.21312)

The neural architecture consists of two cascaded networks:

1. **Convolutional Encoder:** Compresses the input depth/range image into a low-dimensional latent vector.
2. **Coordinate-based MLP:** Approximates the corresponding spatial SDF, parametrizing an explicit position constraint for collision avoidance.

This SDF network is embedded in a velocity-tracking NMPC that outputs **acceleration and yaw rate commands** to the robot. The controller enables reactive collision avoidance directly from sensor observations, without requiring a pre-built map—providing an additional safety layer when odometry drifts or obstacles are incompletely mapped.

**Key Properties:**

- **Mapless navigation:** Operates directly on range images without global map construction.
- **Theoretical guarantees:** Recursive feasibility and stability under fixed observations.
- **Real-time performance:** ~10 ms solve time via acados SQP-RTI solver.
- **Resilience:** Robust to drifting odometry and sensor noise.

## VAE Latent for SDF Representation

We use a two-step process to handle observations:

1. **Encoder (CNN):** Encodes the observation into a compact latent representation, exploiting spatial correlation via convolutions.
2. **Decoder (MLP):** Processes the latent vector concatenated with a 3D query point to predict collision scores.

This architecture provides several advantages:

- **Computational efficiency:** Avoids redundant computation when evaluating the SDF at multiple points given the same observation. Leverages spatial patterns in the input via the CNN, while keeping the MLP lightweight.
- **Robustness:** Improves resilience to noise and systematic errors (e.g., stereo shadows, invalid LiDAR rays). A $\beta$-VAE architecture with a ResNet-10 network as the encoder. It incorporates ReLU activations, batch normalization, and dropout regularization for improved generalizability.

### SDF MLP Architecture

The latent encoding $\mathbf{z}$ is processed using a coordinate-based MLP that approximates the distance transform for the corresponding observation.

**Positional Embedding:** Similar to NeRF-style architectures, positional embedding is first applied to the 3D query point $\mathbf{p}$, mapping it to a high-dimensional space using periodic activation functions. This sinusoidal mapping allows MLPs to better represent high-frequency content.

!!! note "Sensor Specific"
    The SDF MLP operates on spatial data and implicitly encodes the intrinsic projection matrix. It is therefore trained for a specific sensor configuration.

## Problem Formulation

The NMPC solves a velocity-tracking optimal control problem over a receding horizon $T$ with $N$ shooting nodes, subject to learned collision avoidance constraints.

### Collision Avoidance Constraint

Given an observation $o$ encoded into latent $\mathbf{z}$ via the VAE encoder, the SDF network defines $\text{SDF}_{\theta,\mathbf{z}}: \mathbb{R}^3 \to \mathbb{R}$. The collision-free constraint is:

$$\text{SDF}_{\theta,\mathbf{z}}({}^{S_0}\mathbf{p}_B) \geq r + \epsilon$$

where $r$ is the robot-enclosing radius and $\epsilon > 0$ is a user-defined safety margin.

### Field of View Constraints

Since the collision-free set is defined within the sensor frustum $\mathcal{F}(t_0)$, the predicted trajectory must remain within this volume:

$$-\alpha_H \leq \mathcal{S}_{\text{azimuth}}({}^{S_0}\mathbf{p}_S) \leq \alpha_H, \quad -\alpha_V \leq \mathcal{S}_{\text{elevation}}({}^{S_0}\mathbf{p}_S) \leq \alpha_V$$

### Optimal Control Problem

The stage cost $\ell(\mathbf{x}, \mathbf{u})$ penalizes velocity tracking error, heading error, and control effort:

$$\ell(\mathbf{x}, \mathbf{u}) = \left\| \begin{bmatrix} q_{e,z} \\ {}^{V_0}\mathbf{v} - {}^{V_0}\mathbf{v}_{\text{ref}} \end{bmatrix} \right\|^2_Q + \left\| \begin{bmatrix} T_z - mg \\ {}^V\phi \\ {}^V\theta \\ {}^B\omega_z \end{bmatrix} \right\|^2_R$$

where $q_{e,z}$ is the yaw error in quaternion form, $T_z$ is the vertical thrust component, and $Q$, $R$ are tunable positive semidefinite and positive definite weight matrices, respectively.

### Recursive Feasibility

A terminal constraint ensures recursive feasibility by requiring that a "maximum braking to standstill" policy remains feasible from the terminal state. Under fixed observations, this guarantees recursive feasibility and local stability.

## NMPC Topics & Interfaces

Namespace: `/sdf_nmpc/`. Topics are remapped in the launch file.

### Input

| Topic         | Type                                      | Description                                              |
| ------------- | ----------------------------------------- | -------------------------------------------------------- |
| `odometry`    | `nav_msgs/Odometry`                       | State estimate (remapped to `/msf_core/odometry`)        |
| `horizon_ref` | `trajectory_msgs/MultiDOFJointTrajectory` | Reference trajectory from `ref_gen_node.py`              |
| `wps`         | `nav_msgs/Path`                           | Waypoints from planner (remapped to `/gbplanner_path`)   |
| `latent`      | `sdf_nmpc_ros/Latent`                     | VAE latent vector (custom: header + `Float32MultiArray`) |
| `observation` | `sensor_msgs/Image`                       | LiDAR range image (remapped to `/img_node/range_image`)  |

## Output

| Topic              | Type                         | Description                                                     |
| ------------------ | ---------------------------- | --------------------------------------------------------------- |
| `cmd/acc`          | `mavros_msgs/PositionTarget` | Acceleration command (remapped to `/mavros/setpoint_raw/local`) |
| `output/cpt`       | `std_msgs/Float32`           | Solver compute time                                             |
| `output/speed`     | `std_msgs/Float32`           | Current speed                                                   |
| `viz/horizon_traj` | `nav_msgs/Path`              | Predicted horizon trajectory (visualization)                    |

## Services

| Service    | Type               | Description         |
| ---------- | ------------------ | ------------------- |
| `get_flag` | `std_srvs/Trigger` | Get SDF enable flag |
| `set_flag` | `std_srvs/SetBool` | Set SDF enable flag |

## NMPC Configuration

All parameters in `sdf_nmpc_ros/config/<preset>.yaml` (e.g., `sim_lidar.yaml`, `magpie.yaml`).

### Reference

| Parameter      | Description                                          |
| -------------- | ---------------------------------------------------- |
| `ref/yaw_mode` | Yaw control mode (`align`, `ref`, `current`, `zero`) |
| `ref/vref`     | Reference velocity norm (m/s)                        |
| `ref/wzref`    | Reference yaw rate norm (rad/s)                      |
| `ref/zref`     | Desired hovering altitude (m)                        |

### Flags

| Parameter               | Description                           |
| ----------------------- | ------------------------------------- |
| `flags/simulation`      | Simulation or hardware mode (bool)    |
| `flags/enable_sdf`      | Enable collision prediction (bool)    |
| `flags/sdf_cost`        | Use SDF in cost function (bool)       |
| `flags/sdf_constraint`  | Use SDF in constraints (bool)         |
| `flags/vfov_constraint` | Enable vertical FoV constraint (bool) |

### Neural Network

| Parameter        | Description                             |
| ---------------- | --------------------------------------- |
| `nn/size_latent` | Latent vector dimension (typically 128) |
| `nn/vae_device`  | Device for VAE inference (`cuda`)       |
| `nn/sdf_device`  | Device for SDF network (`cuda`)         |
| `nn/vae_weights` | Path to VAE model weights               |
| `nn/sdf_weights` | Path to SDF model weights               |

### MPC

| Parameter               | Description                                                     |
| ----------------------- | --------------------------------------------------------------- |
| `mpc/N`                 | Number of shooting nodes (typically 20)                         |
| `mpc/T`                 | Horizon length (s, typically 1.5)                               |
| `mpc/bound_margin`      | Safety margin for collision constraint (m, typically 0.15)      |
| `mpc/control_loop_time` | Minimum control period (ms, typically 10)                       |
| `mpc/max_solver_fail`   | Maximum successive solver failures before reset (-1 to disable) |

### MPC Weights

| Parameter               | Description                                      |
| ----------------------- | ------------------------------------------------ |
| `mpc/weights/pos`       | Position tracking weights `[px, py, pz]`         |
| `mpc/weights/vel`       | Velocity tracking weights `[vx, vy, vz]`         |
| `mpc/weights/att`       | Attitude tracking weights `[roll, pitch, yaw]`   |
| `mpc/weights/rates`     | Angular rate weights `[wx, wy, wz]`              |
| `mpc/weights/acc`       | Control effort weight                            |
| `mpc/weights/slack_df`  | Slack weights for distance constraint `[L1, L2]` |
| `mpc/weights/slack_fov` | Slack weights for FoV constraint `[L1, L2]`      |

### Robot Limits

| Parameter                     | Description                   |
| ----------------------------- | ----------------------------- |
| `robot/limits/vx`, `vy`, `vz` | Maximum velocities (m/s)      |
| `robot/limits/ax`, `ay`, `az` | Maximum accelerations (m/s²)  |
| `robot/limits/wz`             | Maximum yaw rate (rad/s)      |
| `robot/limits/roll`, `pitch`  | Maximum attitude angles (rad) |

### ROS

| Parameter               | Description                    |
| ----------------------- | ------------------------------ |
| `ros/control_interface` | Control interface type (`acc`) |
| `ros/timeout_ref`       | Reference timeout (s)          |
| `ros/timeout_img`       | Image/observation timeout (s)  |
| `ros/frames/world`      | World frame ID                 |
| `ros/frames/body`       | Body frame ID                  |
| `ros/frames/sensor`     | Sensor frame ID                |

---

## Citation

If you use this code or the associated methods in your research, please cite the following publication:

```bibtex
@misc{navigation_neuralmpc,
	title = {Neural {NMPC} through {Signed} {Distance} {Field} {Encoding} for {Collision} {Avoidance}},
	url = {http://arxiv.org/abs/2511.21312},
	doi = {10.1177/02783649251401223},
	urldate = {2025-11-30},
	author = {Jacquet, Martin and Harms, Marvin and Alexis, Kostas},
	month = nov,
	year = {2025},
}
```
