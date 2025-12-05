## Indicative Results

### Manual Flight (Runehamar Tunnel)

Here an aerial platform with VectorNav VN-100 IMU, Ouster OS0-128 LiDAR, and Texas Instruments IWR6843AOP Radar flying manually through the Runehamar tunnel in Norway. The performance of the LiDAR-radar-inertial fusion can be see in the aggregated point cloud map. Note, the estimate experiences some vertical drift due to difficulties with respect to bias observability, exaggerated by the large distance traveled by the mission.

![runehamar](figures/indicative_results/result_runehamar_hornbill_lri.png)

**Metadata**:

- Distance: 1444 m
- Maximum Velocity: 11 m/s

### Exploration

#### Runehamar Tunnel

In the same environment, we conduct exploration missions with the autonomy stack, executed with the UniPilot module mounted on a quad rotor frame. Here you can see the path it executes, starting at the entrace and continuing onwards into the tunnel, before turning around and returning to home.

![runehamar](figures/indicative_results/result_runehamar_magpie.png)

**Metadata**:

- Distance: 270 m
- Maximum Velocity: 1.9 m/s

#### Løkken Mine

Here we show an exporation mission conducted with the UniPilot module in the Løkken mine. Note, the estimation and planning performance as it transitions from a small, confined corridor out to a large, open space and back again for homing. In the second figure, the aggregated point cloud of the radar can be seen instead. Clearly, the points are significantly noisier than the corresponding map for the LiDAR.

![lokken](figures/indicative_results/result_lokken.png)

![lokken_radar_accumulated](figures/indicative_results/result_lokken_accumulated_radar_cloud.png)

**Metadata**:

- Distance: 197 m
- Maximum Velocity: 1.1 m/s

### Changing the Environment

Although the planner ensures collision free paths, if the environment were to change a given path may no longer be collision free. In this series of experiments, we conduct exploration missions in a cluttered basement corridor, and intentionally sabotage the planner at two distinct points. Both the NMPC- and RL-based safety methods ensure the aerial platform continues safely, when the planners path is compromised.

#### NMPC

Here you can see the NMPC, ensuring safety, by pushing the aerial platform away from the planned path to avoid an object put to cause collision with the path from the planner.

![nmpc](figures/indicative_results/result_nmpc.png)

**Metadata**:

- Distance: 125 m
- Maximum Velocity: 1.8 m/s

#### RL

Here you can see the RL policy, ensuring safety, by pushing the aerial platform away from the planned path to avoid an object put to cause collision with the path from the planner.

![rl](figures/indicative_results/result_rl.png)

**Metadata**:

- Distance: 125 m
- Maximum Velocity: 2.1 m/s

### Legged Robot

The UniPilot with the Unified Autonomy Stack is deployed on the ANYmal D legged robot tasked to explore a part of a university building. The mission involved navigation through both wide open and narrow corridors. The long narrow geometry of the narrow corridors can render a LiDAR only or LiDAR-inertial method to become degenerate. However, through the fusion of radar, the robot is able to resiliently localize and complete the mission.

![anymal](figures/indicative_results/result_anymal.png)

**Metadata**:

- Distance: 873 m
- Maximum Velocity: 1.1 m/s

---

Stay tuned for more evaluations! <img src="../figures/arl_logo.png" alt="ARL Logo" style="height: 2em;">