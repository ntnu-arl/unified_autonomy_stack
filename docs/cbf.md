
# Composite Control Barrier Functions for Last Resort Safety Filtering

Beyond map-based planning and reactive navigation approaches (Neural SDF-NMPC, Exteroceptive-DRL), the Unified Autonomy Stack provides a last-resort safety filter based on Composite Control Barrier Functions (C-CBFs). This filter minimally modifies reference accelerations when an unexpected impending collision is detected, using a sliding window of volumetric map data. Individual obstacle constraints are combined into a composite function that creates a smooth under-approximation of the safe set, with its gradient acting as a "virtual obstacle" to steer the robot away from danger. While designed for rare engagement (as upstream avoidance methods are expected to succeed), this module provides mathematically formal safety guarantees—critical for autonomous robots in demanding environments where neural network-based methods cannot offer strict performance assurances.


!!! note "Source Code"
    - **Workspace:** `workspaces/ws_control/src`
    - **Package:** `composite_cbf`
    - **GitHub:** [ntnu-arl/composite_cbf](https://github.com/ntnu-arl/composite_cbf/tree/dev/unified_autonomy_stack)

**Related Publications**:

- [M. Harms, M. Jacquet and K. Alexis, "Safe Quadrotor Navigation Using Composite Control Barrier Functions," 2025 IEEE International Conference on Robotics and Automation (ICRA), Atlanta, GA, USA, 2025, pp. 6343-6349, doi: 10.1109/icra55743.2025.11127368.](https://ieeexplore.ieee.org/document/11127368)
- [N. Misyats, M. Harms, M. Nissov, M. Jacquet and K. Alexis, "Embedded Safe Reactive Navigation for Multirotors Systems using Control Barrier Functions," 2025 International Conference on Unmanned Aircraft Systems (ICUAS), Charlotte, NC, USA, 2025, pp. 697-704, doi: 10.1109/icuas65942.2025.11007827.](https://ieeexplore.ieee.org/document/11007827)

## CBF Formulation

The safety filter acts on the acceleration setpoint within a standard cascaded position-attitude controller. Given a nominal acceleration $\mathbf{a}_{sp}$, the filter solves an optimization to produce a safe acceleration $\mathbf{a}^*$.

### Obstacle Avoidance Constraint

From $n$ range observations treated as spherical obstacle centroids $\mathbf{p}_1, \ldots, \mathbf{p}_n$, we define position constraints:

$$\nu_{i,0}(\mathbf{x}) \triangleq \|\mathbf{p}_i - \mathbf{p}\|^2 - \epsilon^2$$

Each $\nu_{i,0}$ has relative degree 2, so we introduce ECBFs:

$$\nu_{i,1}(\mathbf{x}) \triangleq \dot{\nu}_{i,0}(\mathbf{x}) - p_0 \nu_{i,0}(\mathbf{x}) = -2\mathbf{v}^\top(\mathbf{p}_i - \mathbf{p}) - p_0 \nu_{i,0}(\mathbf{x})$$

A composite CBF aggregates all $n$ obstacles via log-sum-exp:

$$h(\mathbf{x}) \triangleq -\frac{\gamma}{\kappa} \ln \sum_{i=1}^{n} \exp\left[-\kappa s(\nu_{i,1}(\mathbf{x})/\gamma)\right]$$

where $s = \tanh$ is a saturation function, $\gamma \geq 1$ controls sensitivity, and $\kappa$ controls smoothness.

### Safety Filter QP

The filter solves a soft-constrained QP to minimally modify the nominal command:

$$\mathbf{a}^* = \arg\min_{\mathbf{a} \in \mathbb{R}^3} (\mathbf{a} - \mathbf{a}_{sp})^\top H (\mathbf{a} - \mathbf{a}_{sp}) + \sum_{i=1}^{2} \rho \delta_{fi}$$

subject to:

$$L_g h(\mathbf{x}) \mathbf{a} \geq -L_f h(\mathbf{x}) - \alpha(h(\mathbf{x}))$$

$$L_g h_{fi}(\mathbf{x}) \mathbf{a} \geq -L_f h_{fi}(\mathbf{x}) - \alpha_f h_{fi}(\mathbf{x}) - \delta_{fi} \quad \forall i \in \{1, 2\}$$

$$\delta_{fi} \geq 0 \quad \forall i \in \{1, 2\}$$

where $H$ is a positive-definite weight matrix and $\rho > 0$ is the slack multiplier. The FoV constraints are implemented as soft constraints to avoid feasibility issues from imperfect acceleration tracking and noisy observations.

## Implementation Details

- **Inputs/Outputs:** nominal body acceleration → filtered acceleration
- **Tuning:** `epsilon` (margin), `pole_0` (pole location), `alpha` (decay), `gamma` (sensitivity), `kappa` (smoothness), `clamp_xy/z` (limits)
- **Interface:** runs in ROS; filtered acceleration commanded to PX4 via `mavros`

## CBF Topics & Interfaces

Namespace: `/composite_cbf/`.

### Input

| Topic                        | Type                      | Description                                          |
| ---------------------------- | ------------------------- | ---------------------------------------------------- |
| `/composite_cbf/nominal_cmd` | `geometry_msgs/Twist`     | Nominal acceleration from NMPC or trajectory tracker |
| `/composite_cbf/odometry`    | `nav_msgs/Odometry`       | State estimate                                       |
| `/composite_cbf/obstacles`   | `sensor_msgs/PointCloud2` | Obstacle points from LiDAR processor                 |

### Output

| Topic                               | Type                         | Description                      |
| ----------------------------------- | ---------------------------- | -------------------------------- |
| `/composite_cbf/safe_cmd_twist`     | `geometry_msgs/Twist`        | Filtered acceleration setpoint   |
| `/composite_cbf/safe_cmd_postarget` | `mavros_msgs/PositionTarget` | Direct command to PX4 via MAVROS |
| `/composite_cbf/output_viz`         | `geometry_msgs/TwistStamped` | Output visualization             |
| `/composite_cbf/input_viz`          | `geometry_msgs/TwistStamped` | Input visualization              |

## CBF Configuration

All parameters in `composite_cbf/config/<preset>.yaml` (e.g., `magpie.yaml`).

### Safety Margins

| Parameter | Description                                  |
| --------- | -------------------------------------------- |
| `epsilon` | Barrier function margin (m, typically 0.5)   |
| `pole_0`  | Barrier pole location (typically -2.5)       |
| `kappa`   | Barrier smoothness parameter (typically 80)  |
| `gamma`   | Barrier sensitivity parameter (typically 40) |
| `alpha`   | Barrier decay rate (typically 1.5)           |

### Filtering

| Parameter     | Description                                     |
| ------------- | ----------------------------------------------- |
| `lp_gain_in`  | Low-pass filter gain for input (typically 0.4)  |
| `lp_gain_out` | Low-pass filter gain for output (typically 0.2) |

### Limits

| Parameter  | Description                                                    |
| ---------- | -------------------------------------------------------------- |
| `clamp_xy` | Maximum horizontal acceleration correction (m/s², typically 3) |
| `clamp_z`  | Maximum vertical acceleration correction (m/s², typically 2)   |

### Timing

| Parameter   | Description                               |
| ----------- | ----------------------------------------- |
| `ctrl_freq` | Control loop frequency (Hz, typically 50) |
| `obs_to`    | Obstacle timeout (s, typically 1)         |
| `cmd_to`    | Command timeout (s, typically 1)          |

### Frames

| Parameter          | Description                         |
| ------------------ | ----------------------------------- |
| `output_frame_viz` | Frame ID for visualization messages |

---

## Citation

If you use this method in your work, please cite the following publication:

```bibtex
@inproceedings{navigation_ccbf,
	title = {Safe {Quadrotor} {Navigation} {Using} {Composite} {Control} {Barrier} {Functions}},
	url = {https://ieeexplore.ieee.org/document/11127368},
	doi = {10.1109/ICRA55743.2025.11127368},
	booktitle = {2025 {IEEE} {International} {Conference} on {Robotics} and {Automation} ({ICRA})},
	author = {Harms, Marvin and Jacquet, Martin and Alexis, Kostas},
	year = {2025},
}

@inproceedings{navigation_embedded_ccbf,
  author={Misyats, Nazar and Harms, Marvin and Nissov, Morten and Jacquet, Martin and Alexis, Kostas},
  booktitle={2025 International Conference on Unmanned Aircraft Systems (ICUAS)}, 
  title={Embedded Safe Reactive Navigation for Multirotors Systems using Control Barrier Functions}, 
  year={2025},
  pages={697-704},
  doi={10.1109/ICUAS65942.2025.11007827},
}
```
