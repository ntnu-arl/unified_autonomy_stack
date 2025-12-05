# Navigation & Control

- See [Navigation](#navigation) below.
    - [Neural MPC (NMPC)](nmpc.md)
    - [Reinforcement Learning (RL)](rl.md)
- See [Control (CBF safety layer)](cbf.md) below.

## Navigation

The Unified Autonomy Stack takes a multi-layered approach to safety. Conventionally, safe navigation relied solely on map-based path planning—a single point of failure that could lead to collisions due to odometry drift or incomplete mapping (e.g., thin obstacles). While the stack maintains map-based avoidance as its core approach, it adds reactive safety layers through two complementary methods:

1. **[Neural SDF-NMPC](nmpc.md)** — exteroceptive nonlinear MPC with learned collision constraints, paired with a CBF-based safety filter.
2. **[Exteroceptive DRL](rl.md)** — reinforcement learning policies trained for smooth collision avoidance directly from depth observations.
3. **[Composite CBF Safety Filter](cbf.md)** — a Control Barrier Function filter that modifies commands to ensure safety.

Both methods consume online sensor data and can locally deviate from planned paths when necessary, replacing conventional position controllers that blindly follow map-based trajectories.

## Navigation Module Architecture

![Navigation Control Flow](./report/figs/navigation-stack-diagram.png)

## System Integration & Control Flow

<div class="grid" markdown>

```mermaid
---
title: Aerial Robots
---
graph TB
    Planner["Planning<br/>(GBPlanner3)"]
    Selector{{"Switch<br/>NMPC/RL/Override"}}
    
    Planner -->|Reference<br/>Trajectory| Selector
    
    Selector -->|NMPC Mode| NMPC["NMPC<br/>(ROS 2)"]
    Selector -->|RL Mode| RL["RL Policy<br/>(ROS 2)"]
    
    NMPC -->|Acceleration<br/>Setpoints| CBF["CBF Safety<br/>Filter<br/>(ROS 2)"]
    RL -.->|"Acceleration<br/>Setpoints<br/>(WIP)"| CBF
    
    CBF -->|"Filtered<br/>Acceleration<br/>(MAVROS)"| Autopilot["Onboard<br/>Autopilot"]
    RL -->|"Velocity<br/>Setpoints<br/>(MAVROS)"| Autopilot
    
    Autopilot -->|Motor<br/>Commands| Robot["Multirotor"]
    Selector -->|"Override<br/>Reference<br/>Trajectory<br/>(MAVROS)"| Autopilot

    classDef nmpc fill:#e0e7ff30,stroke:#6366f1,stroke-width:2px,color:#000
    classDef rl fill:#ffedd530,stroke:#f97316,stroke-width:2px,color:#000
    classDef cbf fill:#ede9fe30,stroke:#a855f7,stroke-width:2px,color:#000
    classDef autopilot fill:#cffafe30,stroke:#06b6d4,stroke-width:2px,color:#000
    classDef switch fill:#fef3c730,stroke:#eab308,stroke-width:2px,color:#000
    classDef other fill:#f1f5f930,stroke:#64748b,stroke-width:2px,color:#000
    
    class NMPC nmpc
    class RL rl
    class CBF cbf
    class Autopilot autopilot
    class Selector switch
    class Planner,Robot other
```

```mermaid
---
title: Ground Robots
---
graph TB
    Planner2["Planning<br/>(GBPlanner3)"]
    TrajectoryTracker["Robot<br/>Controller"]
    Robot2["Ground Robot"]
    
    Planner2 -->|Reference<br/>Trajectory| TrajectoryTracker
    TrajectoryTracker -->|Joint Commands| Robot2
    
    classDef planner fill:#f1f5f930,stroke:#64748b,stroke-width:2px,color:#000
    classDef loco fill:#d1fae530,stroke:#10b981,stroke-width:2px,color:#000
    classDef robot fill:#f1f5f930,stroke:#64748b,stroke-width:2px,color:#000
    
    class Planner2 planner
    class TrajectoryTracker loco
    class Robot2 robot
```

</div>

## Low-level Controller

### Real-world Platforms

The onboard controller from the PX4 autopilot is used to track the filtered acceleration or velocity setpoint commands. MAVROS is used to communicate these commands to the PX4 firmware.

### Simulation

In simulation, geometric controllers based on [T. Lee et. al., "Control of Complex Maneuvers for a Quadrotor UAV using Geometric Methods on SE(3)"](https://arxiv.org/abs/1003.2005) are are used in Gazebo.
