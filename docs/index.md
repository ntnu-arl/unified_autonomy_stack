# Unified Autonomy Stack

Welcome to the documentation for the **Unified Autonomy Stack**. 
This stack presents an autonomy architecture integrating perception, planning, and navigation algorithms developed and field tested at the <img src="figures/arl_logo.png" alt="ARL Logo" style="height: 1.5em; middle;"> [Autonomous Robots Lab](https://www.autonomousrobotslab.com) across robot configurations. The stack consists of the software for the core algorithms along with drivers, utilities, and tools for simulation and testing. We currently support rotary-wing (e.g., multirotor) and certain ground systems (e.g., legged robots) with extension to other configurations, such as underwater robots, coming soon. The software distributed as a part of this stack has been thoroughly tested in real-world scenarios, demonstrating robust autonomous operation in challenging GPS-denied environments.

![Unified Autonomy Stack Landing Image](report/figs/merged-multi-domain-photo.jpg)

## Overview

The `Unified Autonomy Stack` is designed to provide a robust and flexible foundation for autonomous operations in various environments. It features:

- **Multi-modal Perception**: fusing LiDAR, radar, vision, and IMU data for robust Simultaneous Localization and Mapping (SLAM) alongside integration of Vision-Language Models (VLMs) for high-level interaction.
- **Planning**: Graph-based efficient path planning algorithms tailored for volumetric exploration, visual inspection, and waypoint navigation in complex environments. The planning framework extends to aerial, ground, and underwater robots.
- **Multi-layered Safe Navigation**: Combining map-based path planning with learning-based reactive navigation and safety layers.
    - **SDF-NMPC and RL**: Neural MPC and Reinforcement Learning based map-free approaches for safe navigation.
    - **Last-resort Safety**: Control Barrier Functions for filtering unsafe commands.
- **Multi-platform Support**: Designed for both aerial robots and ground robots with planned extension to underwater.
- **Containerized Deployment**: Docker-based deployment for easy setup and reproducibility across different platforms

## Getting Started

Please navigate through the tabs to explore setup:

- **[Installation](installation.md)**: Instructions for installation and setup.
- **[Deployment](deployment.md)**: Docker-based deployment instructions.
- **[Examples](examples.md)**: Examples for testing the stack in simulation and on datasets.

descriptions of the subsystems:

- **[Architecture](architecture.md)**: High-level overview of the system components and data flow.
- **[Multi-modal SLAM](slam.md)**: Details on the estimation stack.
- **[VLM](vlm.md)**: Details on the VLM stack.
- **[Planning](planning.md)**: Explanation of the planning stack.
- **[Navigation](navigation.md)**: Information on [Neural MPC](nmpc.md), [RL](rl.md) and [Composite CBF Safety Filter](cbf.md) for safe navigation.
- **[Simulation](simulation.md)**: Simulation environments and setup.
- **[Multi-platform Support](platforms.md)**: Verified on diverse aerial and ground robots with the ambition to eventually cover most morphologies across air, land and sea.

and indicative results and datasets:

- **[Prior Results](prior_results.md)**: Previous experiences which create the foundation for the Unified Autonomy Stack.
- **[Indicative Results](indicative_results.md)**: Results of the Unified Autonomy Stack on real robots.
- **[Datasets](datasets.md)**: Relevant datasets for offline evaluation.

## Technical Report

For a comprehensive report of the `Unified Autonomy Stack` please refer to the [Technical Report](./report/UnifiedAutonomyStack-Outline.md){data-preview}.

## Contact

- Mihir Dharmadhikari: [mihir.dharmadhikari@ntnu.no](mailto:mihir.dharmadhikari@ntnu.no)
- Nikhil Khedekar: [nikhil.v.khedekar@ntnu.no](mailto:nikhil.v.khedekar@ntnu.no)
- Mihir Kulkarni: [mihir.kulkarni@ntnu.no](mailto:mihir.kulkarni@ntnu.no)
- Morten Nissov: [morten.nissov@ntnu.no](mailto:morten.nissov@ntnu.no)
- Angelos Zacharia: [angelos.zacharia@ntnu.no](mailto:angelos.zacharia@ntnu.no)
- Martin Jacquet: [martin.jacquet@ntnu.no](mailto:martin.jacquet@ntnu.no)
- Albert Gassol Puigjaner : [albert.g.puigjaner@ntnu.no](mailto:albert.g.puigjaner@ntnu.no)
- Philipp Weiss: [philipp.weiss@ntnu.no](mailto:philipp.weiss@ntnu.no)
- Kostas Alexis: [konstantinos.alexis@ntnu.no](mailto:konstantinos.alexis@ntnu.no)
