---
id: ros2-core-module-1
title: "Module 1: ROS 2 Core"
sidebar_position: 1
---

# Module 1: ROS 2 Core

## Introduction

ROS 2 (Robot Operating System 2) plays a pivotal role in the development of Physical AI systems. It provides a flexible framework for writing robot software across a wide range of platforms, from small embedded systems to large-scale, multi-robot deployments. This module will introduce the core concepts of ROS 2, focusing on how it enables complex robotic behaviors through standardized communication, hardware abstraction, and a rich ecosystem of tools.

## Robot Kinematics and URDF Structure

In robotics, accurately describing the physical properties and kinematic relationships of a robot is crucial for simulation, visualization, and control. ROS 2 leverages two primary formats for this: **URDF** (Unified Robot Description Format) and **Xacro** (XML Macros for ROS).

- **URDF**: An XML format for describing all aspects of a robot, including its visual appearance, collision properties, kinematic tree (links and joints), and inertial properties. It's a static description, meaning once loaded, the robot's fundamental structure doesn't change.
- **Xacro**: A macro language that allows for more concise and readable URDF files by enabling the use of variables, mathematical expressions, and reusable macros. This prevents repetition and makes complex robot descriptions more manageable. Xacro files are typically processed into standard URDF before being used by ROS 2 tools.

### Links and Joints

The fundamental building blocks of a robot's kinematic chain in URDF are **links** and **joints**:

-   **Link**: Represents a rigid body of the robot. This could be a wheel, a chassis, a robot arm segment, or any other solid component. Links have associated visual properties (how they look), collision properties (how they interact physically), and inertial properties (mass, center of mass, inertia matrix).
-   **Joint**: Defines the kinematic and dynamic connection between two links. A joint specifies how a child link can move relative to its parent link. Joints can have different types (e.g., `fixed`, `revolute`, `prismatic`) that dictate their degrees of freedom.

Let's illustrate with a basic differential drive robot, starting with its `base_link` and one `revolute` joint for a wheel.

```xml
<?xml version="1.0"?>
<robot name="differential_robot">

  <!-- BASE LINK -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.4 0.2 0.1"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 0.8 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.4 0.2 0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="5.0"/>
      <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1"/>
    </inertial>
  </link>

  <!-- LEFT WHEEL JOINT -->
  <joint name="left_wheel_joint" type="revolute">
    <parent link="base_link"/>
    <child link="left_wheel_link"/>
    <origin xyz="-0.15 0.12 0" rpy="1.57075 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1000" upper="1000" effort="100" velocity="100"/>
  </joint>

  <!-- LEFT WHEEL LINK (defined for the joint) -->
  <link name="left_wheel_link">
    <visual>
      <geometry>
        <cylinder radius="0.05" length="0.04"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.05" length="0.04"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

</robot>
```

In this example:
-   The `<link name="base_link">` defines the main chassis of the robot.
-   The `<joint name="left_wheel_joint" type="revolute">` connects the `base_link` (parent) to the `left_wheel_link` (child). The `type="revolute"` indicates that this joint allows rotation around a single axis, simulating a wheel.
-   The `<origin>` tag specifies the joint's position (`xyz`) and orientation (`rpy` - roll, pitch, yaw) relative to the parent link. Here, `rpy="1.57075 0 0"` rotates the wheel to be vertical.
-   The `<axis xyz="0 0 1"/>` defines the axis of rotation for the `revolute` joint (in this case, around the Z-axis of the joint's local frame).

---

## ROS 2 Package Setup and RViz 2 Visualization

To effectively manage robot software and integrate with the ROS 2 ecosystem, understanding how to create and structure ROS 2 packages is essential. This section will guide you through setting up a workspace, creating a package for your robot description, and visualizing it in RViz 2 using a launch file.

### 1. Creating a ROS 2 Workspace and Package

A ROS 2 workspace is a directory where you store and build your ROS 2 packages. We'll use `colcon` for building.

**Step 1: Create a Workspace**

First, create a new directory for your ROS 2 workspace and navigate into it:

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

**Step 2: Create a New ROS 2 Package**

Now, create a new ROS 2 package called `ros2_robot_description` using the `ros2 pkg create` command. We'll specify `rclpy` and `launch_ros` as build and run dependencies, as we'll be using Python for nodes and launch files.

```bash
ros2 pkg create --build-type ament_python ros2_robot_description --dependencies rclpy launch_ros
```

This command creates a new directory `ros2_robot_description` with the basic structure for a Python ROS 2 package.

### 2. `package.xml` and `CMakeLists.txt` for Robot Description

For a package containing robot descriptions (like URDFs), the `package.xml` and `CMakeLists.txt` (or `setup.py` for Python) need to be configured correctly.

**`package.xml` Requirements:**

The `package.xml` defines the package metadata and its dependencies. For a robot description package, you typically need `urdf`, `xacro` (if using xacro), `robot_state_publisher`, and `joint_state_publisher` as dependencies. Ensure these are listed:

```xml
<dependency>urdf</dependency>
<dependency>xacro</dependency>
<exec_dependency>robot_state_publisher</exec_dependency>
<exec_dependency>joint_state_publisher</exec_dependency>
```

**Python `CMakeLists.txt` (`setup.py`) for Robot Description:**

For Python packages, `setup.py` is used to define how the package is installed. You'll need to ensure your URDF and launch files are installed correctly. An example snippet for `setup.py` to install URDF and launch files might look like this:

```python
from setuptools import setup
import os
from glob import glob

package_name = 'ros2_robot_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
        (os.path.join('share', package_name, 'config'), glob('config/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='ROS 2 package for robot description',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
```
*Note: You would place your URDF files in `ros2_robot_description/urdf/` and launch files in `ros2_robot_description/launch/`.*

### 3. Visualizing in RViz 2 with a ROS 2 Launch File

ROS 2 launch files (typically Python or XML) are used to start multiple nodes simultaneously, often configuring their parameters and setting up the entire robot system. We will create a launch file to load our URDF and visualize it in RViz 2.

**Step 1: Create the URDF File**

Inside your `ros2_robot_description` package, create a `urdf` directory and place your URDF file (e.g., `robot.urdf`) there. For example:

```xml
<!-- ros2_robot_description/urdf/robot.urdf -->
<?xml version="1.0"?>
<robot name="simple_robot">
  <link name="base_link">
    <visual>
      <geometry><box size="0.2 0.2 0.2"/></geometry>
    </visual>
  </link>
</robot>
```

**Step 2: Create a Launch File**

Create a `launch` directory inside your `ros2_robot_description` package and add a Python launch file (e.g., `display_robot.launch.py`):

```python
# ros2_robot_description/launch/display_robot.launch.py
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution

def generate_launch_description():
    # Declare arguments
    urdf_model_path = os.path.join(
        get_package_share_directory('ros2_robot_description'),
        'urdf',
        'robot.urdf'
    )
    rviz_config_path = os.path.join(
        get_package_share_directory('ros2_robot_description'),
        'config',
        'robot.rviz'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            name='model',
            default_value=TextSubstitution(text=urdf_model_path),
            description='Path to robot URDF file'
        ),
        DeclareLaunchArgument(
            name='rvizconfig',
            default_value=TextSubstitution(text=rviz_config_path),
            description='Path to RViz config file'
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': LaunchConfiguration('model')}]
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', LaunchConfiguration('rvizconfig')]
        )
    ])
```

**Step 3: Create RViz Configuration File**

Create a `config` directory inside your `ros2_robot_description` package and add an RViz configuration file (e.g., `robot.rviz`). This file will define what RViz displays. For a basic visualization, you'll need at least `RobotModel` and `JointState` displays. You can create a default one by launching RViz manually, configuring it, and then saving the configuration.

**Step 4: Build and Launch**

Navigate back to your workspace root (`~/ros2_ws`) and build your package:

```bash
cd ~/ros2_ws
colcon build --packages-select ros2_robot_description
```

After building, source your workspace and launch the display file:

```bash
source install/setup.bash
ros2 launch ros2_robot_description display_robot.launch.py
```

This will launch `robot_state_publisher` (which broadcasts the robot's state from URDF), `joint_state_publisher_gui` (to manually control joints for demonstration), and `rviz2` (to visualize the robot).

---

## Joint State Publisher and Manual Control

To visualize a robot model with articulated joints, ROS 2 relies on a combination of nodes that publish the robot's state. This section explains the roles of the `joint_state_publisher` and `robot_state_publisher` nodes, demonstrates how to manually control joints using `joint_state_publisher_gui`, and shows how to orchestrate everything with an updated launch file.

### 1. The Role of Joint State Publisher and Robot State Publisher

-   **`joint_state_publisher`**: This node takes the values of the non-fixed joints in a robot (e.g., revolute, prismatic) and publishes them as a single `sensor_msgs/msg/JointState` message on the `/joint_states` topic. It can either read these values from parameters, or, as we'll see, provide a GUI for manual control.
-   **`robot_state_publisher`**: This node subscribes to `sensor_msgs/msg/JointState` messages (typically from `joint_state_publisher`) and transforms them into `tf2` transformations. These transformations represent the kinematic chain of the robot, allowing RViz 2 to display the robot's links and joints in their correct relative positions and orientations.

### 2. Manual Joint Control with `joint_state_publisher_gui`

The `joint_state_publisher_gui` is an interactive tool that allows you to manually manipulate the joint angles of your robot model and visualize the changes in real-time within RViz 2. It publishes joint states, which are then used by `robot_state_publisher` to update the robot's `tf2` tree.

To use it, you typically include it in your launch file. Once launched, a GUI window will appear, allowing you to slide individual joint values. As you change these values, `joint_state_publisher_gui` publishes new `/joint_states` messages, and RViz 2 updates accordingly.

### 3. Orchestrating with an Updated Launch File

To bring everything together for a complete visualization pipeline – loading the URDF, publishing joint states (manual or programmatic), broadcasting robot transformations, and launching RViz 2 – we'll update our launch file to include all necessary nodes. The launch file previously shown in the "ROS 2 Package Setup and RViz 2 Visualization" section already includes these nodes, but let's review the key components for clarity:

```python
# ... (imports and path declarations as before)

def generate_launch_description():
    # ... (urdf_model_path and rviz_config_path declarations as before)

    return LaunchDescription([
        # ... (DeclareLaunchArgument for model and rvizconfig as before)

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': LaunchConfiguration('model')}]
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', LaunchConfiguration('rvizconfig')]
        )
    ])
```

**Explanation of Nodes in the Launch File:**

-   **`robot_state_publisher`**: This node reads the robot description (URDF) provided as a parameter (`robot_description`) and publishes the static and dynamic joint transformations to `/tf` and `/tf_static` topics, which are consumed by RViz 2.
-   **`joint_state_publisher_gui`**: This node provides a GUI to control the values of non-fixed joints. It publishes `sensor_msgs/msg/JointState` messages to the `/joint_states` topic.
-   **`rviz2`**: The primary visualization tool. It subscribes to `/tf` and `/joint_states` (indirectly via `robot_state_publisher`) to display the robot model.

By launching these three nodes together, you create a complete visualization pipeline where you can manipulate your robot's joints manually and observe the kinematic changes in RViz 2.

---

## Core ROS 2 CLI Tools

ROS 2 provides a powerful command-line interface (CLI) to interact with and inspect your robot system. These tools are indispensable for debugging, monitoring, and understanding the behavior of your ROS 2 applications. This section will guide you through the most commonly used CLI commands.

### 1. Running Nodes: `ros2 run`

The `ros2 run` command is used to launch an executable node within a ROS 2 package. Its basic syntax is:

```bash
ros2 run <package_name> <executable_name>
```

For example, to run the `joint_state_publisher_gui` from the `joint_state_publisher_gui` package:

```bash
ros2 run joint_state_publisher_gui joint_state_publisher_gui
```

### 2. Topic Inspection: `ros2 topic`

Topics are the primary means of communication in ROS 2. The `ros2 topic` command allows you to inspect, publish, and subscribe to topics.

-   **`ros2 topic list`**: Lists all active topics in the ROS 2 graph.

    ```bash
    ros2 topic list
    ```

    Expected output might include `/joint_states`, `/tf`, `/tf_static`, etc.

-   **`ros2 topic info <topic_name>`**: Displays information about a specific topic, including its type and the nodes that publish/subscribe to it.

    ```bash
    ros2 topic info /joint_states
    ```

-   **`ros2 topic echo <topic_name>`**: Prints the messages being published on a given topic to the console.

    ```bash
    ros2 topic echo /joint_states
    ```

    This is extremely useful for debugging, allowing you to see the raw data being exchanged.

### 3. Node Inspection: `ros2 node`

Nodes are the executables in ROS 2 that perform computation. The `ros2 node` command helps you manage and inspect running nodes.

-   **`ros2 node list`**: Lists all active nodes in the ROS 2 graph.

    ```bash
    ros2 node list
    ```

    You might see `/robot_state_publisher`, `/joint_state_publisher_gui`, `/rviz2`, etc.

-   **`ros2 node info <node_name>`**: Provides detailed information about a specific node, including its published topics, subscribed topics, services, and parameters.

    ```bash
    ros2 node info /robot_state_publisher
    ```

### 4. Parameter Management: `ros2 param`

Parameters in ROS 2 allow nodes to expose configurable values. The `ros2 param` command enables you to inspect and modify these parameters at runtime.

-   **`ros2 param list <node_name>`**: Lists all parameters exposed by a specific node.

    ```bash
    ros2 param list /robot_state_publisher
    ```

-   **`ros2 param get <node_name> <parameter_name>`**: Retrieves the current value of a specific parameter from a node.

    ```bash
    ros2 param get /robot_state_publisher robot_description
    ```

-   **`ros2 param set <node_name> <parameter_name> <value>`**: Sets the value of a specific parameter on a running node.

    ```bash
    # Example: (Note: Not all parameters can be set dynamically)
    # ros2 param set /some_node some_param_name new_value
    ```

By mastering these CLI tools, you gain powerful capabilities to monitor, debug, and interact with your ROS 2 robot systems efficiently.

---

## Module 1 Assessment: The Kinematics Challenge

This module has introduced you to the foundational elements of ROS 2 for Physical AI, covering the critical aspects of robot description, kinematics, package management, and command-line tools. You've learned to:

-   **Define Robot Structure with URDF/Xacro**: How to use XML-based formats to represent robot links and joints, including their visual, collision, and inertial properties.
-   **Understand Kinematics**: The basics of how joints connect links and the principles governing robot motion.
-   **Create and Manage ROS 2 Packages**: The structure and tools (`ros2 pkg create`, `package.xml`, `setup.py`) for organizing your robot software.
-   **Visualize Robots with RViz 2 and Launch Files**: How to use ROS 2 launch files to orchestrate `robot_state_publisher`, `joint_state_publisher_gui`, and RViz 2 for real-time robot visualization and manual joint control.
-   **Utilize ROS 2 CLI Tools**: Essential commands like `ros2 run`, `ros2 topic`, `ros2 node`, and `ros2 param` for inspecting and debugging your ROS 2 system.

### The Challenge: A Two-Wheeled Robot with a Rotating Sensor

Your task is to extend the basic differential drive robot to include a new rotating sensor on top of its chassis. This challenge will test your understanding of URDF, kinematics, and ROS 2 visualization tools.

**Requirements**:

1.  **Modify URDF**: Extend your `robot.urdf` file to include:
    -   A new `caster_wheel_link` (e.g., a small sphere) connected to the `base_link` with a `fixed` joint at the front of the robot. This simulates a passive caster wheel.
    -   A new `sensor_link` (e.g., a small cylinder or box) mounted on top of the `base_link`.
    -   A new `revolute` joint, `sensor_joint`, connecting the `base_link` to the `sensor_link`. This joint should allow the sensor to rotate around its vertical axis (Z-axis).

2.  **Verify Kinematics**: Ensure your `sensor_joint` has appropriate `<origin>` and `<axis>` tags so that the sensor rotates correctly relative to the `base_link`.

3.  **Visualize in RViz 2**: Launch your updated robot model in RViz 2. Confirm that:
    -   The `caster_wheel_link` and `sensor_link` are visible and correctly positioned.
    -   You can manually control the `sensor_joint`'s rotation using `joint_state_publisher_gui`, and the `sensor_link` rotates as expected in RViz 2.

4.  **Inspect with CLI**: While your robot visualization is running, use `ros2 topic echo /joint_states` to observe the `sensor_joint`'s position changes as you manipulate it via `joint_state_publisher_gui`. Take a screenshot or record a short video demonstrating this.

This challenge encourages you to apply the concepts learned in this module to build a more complex robot description and debug its kinematic behavior using ROS 2 tools. Good luck!


To effectively engage with this module, ensure you have the following:

- [ ] A working installation of ROS 2 (Foxy, Galactic, Humble, or Rolling recommended) on a Linux system (Ubuntu 20.04+ preferred).
- [ ] Basic familiarity with Linux command-line operations.
- [ ] Fundamental understanding of C++ or Python programming.
- [ ] A text editor or IDE (e.g., VS Code).
- [ ] Internet access to fetch ROS 2 packages and documentation.
