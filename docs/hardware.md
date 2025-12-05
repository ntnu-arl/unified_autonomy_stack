## Included Hardware and Drivers

This section provides an overview of the hardware components and drivers that are compatible with the Unified Autonomy Stack. It includes details on supported sensors, actuators, and communication interfaces, as well as instructions for integrating new hardware into the framework.



### Supported Hardware

We currently support the hardware components mounted on the UniPilot autonomy payload, which has been specifically designed for seamless integration with the Unified Autonomy Stack.

![Unipilot Hardware](figures/unipilot/unipilot.png)

This hardware is listed in this section is tested and verified to work with an [NVIDIA Jetson Orin NX](https://www.nvidia.com/en-us/autonomous-machines/embedded-systems/jetson-orin/) 16GB compute module with a [ConnectTech Boson 22](https://connecttech.com/product/boson-22-carrier-board-for-nvidia-jetson-orin-nano/) (MIPI) Carrier Board.

The list of supported hardware includes:

- **LiDAR Sensors**: Robosense Airy LiDAR sensor
!!! note "LiDAR Driver Source Code"
    - **Workspace:** `workspaces/ws_rslidar_sdk/src`
    - **Package:** `rslidar_sdk`
    - **GitHub:** [ntnu-arl/rslidar_sdk](https://github.com/ntnu-arl/rslidar_sdk)
- **Radar Sensors**: D3 Embedded RS-6843AOPU
!!! note "Radar Driver Source Code"
    - **Workspace:** `workspaces/ws_mmwave_ti_ros/src`
    - **Package:** `ti_mmwave_rospkg`
    - **GitHub:** [ti/mmwave_ti_ros](https://git.ti.com/git/mmwave_radar/mmwave_ti_ros.git) (with custom patch)
- **Cameras**: Vision Components IMX296 MIPI Cameras
!!! note "Camera Driver Source Code"
    - **Workspace:** `workspaces/ws_ros_gst_bridge/src`
    - **Package:** `ros-gst-bridge`
    - **GitHub:** [ntnu-arl/ros-gst-bridge](https://github.com/ntnu-arl/ros-gst-bridge)
- **Camera Synchronization**: Custom electronics for hardware triggering and synchronization of multiple cameras
!!! note "Camera Sync Source Code"
    - **Workspace:** `workspaces/ws_cam_sync/src`
    - **Package:** `cam_sync`
    - **GitHub:** [ntnu-arl/cam_sync](https://github.com/ntnu-arl/cam_sync)
- **Time of Flight (ToF) Sensor**: pmd PicoFlexx 2 ToF Camera
!!! note "ToF Driver Source Code"
    - **Workspace:** `workspaces/ws_royale_in_ros/src`
    - **Package:** `royale_in_ros`
    - **GitHub:** [ntnu-arl/royale_in_ros](https://github.com/ntnu-arl/royale_in_ros)
- **Inertial Measurement Unit (IMU)**: VectorNav VN-100 IMU
!!! note "IMU Driver Source Code"
    - **Workspace:** `workspaces/ws_vectornav/src`
    - **Package:** `vectornav_driver`
    - **GitHub:** [ntnu-arl/vectornav](https://github.com/ntnu-arl/vectornav)
- **Flight Controller**: PX4 Autopilot
!!! note "Flight Controller Interface Source Code"
    - **Workspace:** `workspaces/ws_mavros/src`
    - **Package:** `mavros`
    - **GitHub:** [ntnu-arl/mavros](https://github.com/ntnu-arl/mavros)
