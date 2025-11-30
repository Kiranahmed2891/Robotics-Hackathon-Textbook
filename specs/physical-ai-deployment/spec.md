---
sidebar_position: 3
title: Module 3 - Physical AI and Deployment
---

# Module 3: Physical AI and Deployment - Testable Specifications

This document outlines testable specifications for Module 3: Physical AI and Deployment, covering Jetson setup, Sim-to-Real parity, real-time SLAM, E-stop safety, and modular launch files. These specifications will guide the implementation and verification of the physical robot's capabilities and deployment processes.

## Specifications

### 1. Jetson Setup (ROS 2)
*   **SPEC ID**: `002-S1`
*   **Description**: The NVIDIA Jetson board shall be configured with a minimal Ubuntu 20.04 (or later) OS, ROS 2 Foxy (or later) installed from binaries, and necessary dependencies for the robot's software stack.
*   **Acceptance Criteria**:
    *   **2.1.1** The Jetson device successfully boots into the specified Ubuntu OS.
    *   **2.1.2** ROS 2 Foxy (or later) is installed and sourced correctly, allowing `ros2` commands to execute without error.
    *   **2.1.3** All required ROS 2 packages (e.g., `ros2_control`, `navigation2`, `slam_toolbox`) are installed and accessible on the Jetson.
    *   **2.1.4** The robot's custom workspace (containing URDF, controllers, launch files) successfully builds on the Jetson without compilation errors.

### 2. Sim-to-Real Parity
*   **SPEC ID**: `002-S2`
*   **Description**: The robot's simulated behavior in Gazebo shall closely match its physical behavior in the real world under equivalent control commands and environmental conditions.
*   **Acceptance Criteria**:
    *   **2.2.1** When given identical velocity commands, the physical robot's linear and angular speeds deviate by no more than ±5% from the simulated robot's speeds over a 5-second interval.
    *   **2.2.2** The physical robot's odometry data (position and orientation) matches the simulated robot's odometry within a 10% margin of error after traversing a 2-meter straight line and a 90-degree turn.
    *   **2.2.3** Sensor readings (LiDAR ranges, camera images) from the physical robot accurately reflect the real environment compared to the simulated sensor data in a matched virtual environment.
    *   **2.2.4** The robot's control system (e.g., differential drive controller) performs comparably on both simulated and physical hardware, maintaining stability and responsiveness.

### 3. Real-time SLAM
*   **SPEC ID**: `002-S3`
*   **Description**: The robot shall perform real-time Simultaneous Localization and Mapping (SLAM) using its onboard LiDAR sensor, generating a consistent 2D occupancy grid map and providing accurate self-localization within that map.
*   **Acceptance Criteria**:
    *   **2.3.1** The `slam_toolbox` (or equivalent SLAM package) successfully launches on the Jetson and subscribes to LiDAR and odometry topics.
    *   **2.3.2** A 2D occupancy grid map is continuously generated and updated in RViz (or similar visualization tool) as the robot explores a new environment.
    *   **2.3.3** The robot's localized pose in the generated map remains consistent (drift less than 5cm over 5 meters of travel) when traversing a known path.
    *   **2.3.4** Loop closures are detected and correctly applied, improving map consistency and localization accuracy in explored areas.
    *   **2.3.5** The SLAM process maintains a real-time update rate (e.g., map updates at least 5 Hz, pose updates at least 30 Hz) on the Jetson.

### 4. E-Stop Safety
*   **SPEC ID**: `002-S4`
*   **Description**: An emergency stop (E-stop) system shall be implemented, capable of immediately halting all robot motion and disabling actuators upon activation, ensuring fail-safe operation.
*   **Acceptance Criteria**:
    *   **2.4.1** Activation of the physical E-stop button (or software E-stop command) immediately stops all wheel motors and any other actuators within 0.1 seconds.
    *   **2.4.2** Once activated, the robot remains immobile and unresponsive to control commands until the E-stop is physically reset.
    *   **2.4.3** The E-stop status is clearly indicated visually (e.g., LED) on the robot and published on a dedicated ROS 2 topic (e.g., `/e_stop_status`).
    *   **2.4.4** The E-stop system is independent of the main control software stack, ensuring it functions even if the primary control system fails.

### 5. Modular Launch Files
*   **SPEC ID**: `002-S5`
*   **Description**: The deployment of the robot's software stack shall use a modular ROS 2 launch system, allowing easy activation/deactivation of components and adaptation to different operational scenarios.
*   **Acceptance Criteria**:
    *   **2.5.1** A top-level launch file (e.g., `robot_bringup.launch.py`) exists that can start all core robot functionalities (e.g., `robot_state_publisher`, `controllers`, `SLAM`).
    *   **2.5.2** Individual components (e.g., `SLAM`, `navigation`, `sensors`) can be launched or excluded via command-line arguments or environment variables within the launch system.
    *   **2.5.3** Launch files are organized logically within the robot's ROS 2 packages (e.g., `robot_description/launch`, `robot_navigation/launch`).
    *   **2.5.4** The launch system allows for easy switching between simulation and real-robot modes without modifying core configuration files (e.g., using a `use_sim_time` parameter or conditional includes).
    *   **2.5.5** New functionalities or sensors can be added to the launch system by creating new modular launch files without requiring extensive changes to existing ones.
