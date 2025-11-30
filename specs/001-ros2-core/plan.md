# Implementation Plan: ROS 2 Core Module 1: URDF, Kinematics, Packages, Best Practices

**Branch**: `001-ros2-core` | **Date**: 2025-11-30 | **Spec**: [specs/001-ros2-core/spec.md]
**Input**: Feature specification from `/specs/001-ros2-core/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

This plan outlines the content architecture and design for Module 1 of ROS 2 Core, focusing on URDF structure, joint kinematics, package creation, and adherence to best practices, as defined in the associated feature specification. The technical approach involves leveraging ROS 2 tools and libraries to model robot systems, simulate their movements, and organize development efficiently.

## Technical Context

**Language/Version**: C++ (ROS 2 Foxy/Rolling) and Python (3.8+)  
**Primary Dependencies**: ROS 2 core packages (rclcpp, rclpy, ament_cmake, ament_python), robot_state_publisher, joint_state_publisher, kinematics solvers (e.g., KDL, MoveIt! for advanced cases)  
**Storage**: N/A (for core module, persistent storage not primarily handled)  
**Testing**: ament_cmake_gtest (C++), ament_cmake_pytest (Python)  
**Target Platform**: Linux (Ubuntu 20.04+ or other ROS 2 supported distributions)
**Project Type**: Robotics framework extension/development (ROS 2 package-based)  
**Performance Goals**: URDF visualization within 5 minutes (SC-001), forward kinematics calculation within 100 milliseconds (SC-002)  
**Constraints**: Adherence to ROS 2 architectural guidelines (REP), real-time capability where applicable for robot control, strict adherence to package dependency management.  
**Scale/Scope**: Focus on fundamental concepts for single to small multi-robot systems within a ROS 2 environment.

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

- **I. Library-First**: Pass (ROS 2 packages inherently promote modular, library-first development)
- **II. CLI Interface**: Pass (ROS 2 tools and nodes are primarily CLI-driven)
- **III. Test-First (NON-NEGOTIABLE)**: Pass (ROS 2 development strongly encourages testing via `ament_cmake_gtest`/`pytest`)
- **IV. Integration Testing**: Pass (Implicitly covered by ROS 2 system testing and simulation)
- **V. Observability**: Pass (ROS 2 provides robust logging, metrics via `ros2 topic/node/param` tools)
- **VI. Versioning & Breaking Changes**: Pass (ROS 2 uses semantic versioning; package.xml)
- **VII. Simplicity**: Pass (ROS 2 encourages modularity and focused packages to maintain simplicity)

## Project Structure

### Documentation (this feature)

```text
specs/001-ros2-core/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
ros2_core_module/
├── src/
│   └── ros2_core_module/
│       └── (C++/Python source files for nodes, libraries)
├── include/
│   └── ros2_core_module/
│       └── (C++ headers)
├── launch/
│   └── (ROS 2 launch files)
├── config/
│   └── (ROS 2 parameter/configuration files)
├── urdf/
│   └── (URDF and XACRO robot description files)
├── CMakeLists.txt
├── package.xml
└── test/
    ├── (gtest C++ tests)
    └── (pytest Python tests)
```

**Structure Decision**: The single project structure is chosen, organized as a standard ROS 2 package. This allows for modular development of nodes, libraries, and robot descriptions within a well-defined structure. The example above outlines the standard directories for source code, headers, launch files, configurations, URDF models, and tests.

## Complexity Tracking

