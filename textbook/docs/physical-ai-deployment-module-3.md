---
sidebar_position: 3
title: Module 3 - Physical AI and Deployment
---

# Module 3: Physical AI and Deployment

## Introduction: Bringing AI to the Physical World

Welcome to Module 3: Physical AI and Deployment. In this module, we transition from the simulated environment to the real world, focusing on deploying our robot's AI capabilities onto physical hardware. This involves setting up embedded systems like the NVIDIA Jetson Orin, ensuring seamless Sim-to-Real parity, implementing real-time Simultaneous Localization and Mapping (SLAM), integrating robust safety protocols (E-Stop), and structuring our software with modular launch files for flexible deployment. The goal is to bridge the gap between simulation and reality, enabling our robot to perceive, navigate, and interact autonomously in physical environments.

## S1: Jetson Orin ROS 2 Setup Guide

To prepare our NVIDIA Jetson Orin for robot deployment, we need a robust and minimal ROS 2 environment. This section details the step-by-step process of configuring the Jetson with the necessary operating system, ROS 2 distribution, and essential development tools.

### 1.1 Install Ubuntu 20.04 (or later)

Ensure your Jetson Orin Developer Kit is flashed with a compatible Ubuntu image. NVIDIA provides specific SDK Manager tools for this. Follow the official NVIDIA documentation for installing the Jetson OS, making sure to select Ubuntu 20.04 (or a later LTS version).

### 1.2 Install ROS 2 Foxy (or later) from Binaries

Follow the official ROS 2 documentation for installing Foxy (or your chosen distribution) from binaries. This ensures a stable and tested ROS 2 environment. The general steps are:

1.  **Set up locales**:
    ```bash
    locale  # check for UTF-8
    sudo apt update && sudo apt install locales
    sudo locale-gen en_US en_US.UTF-8
    sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
    export LANG=en_US.UTF-8
    locale  # verify settings
    ```

2.  **Setup Sources**: Add the ROS 2 apt repository.
    ```bash
    sudo apt update && sudo apt install curl gnupg2 lsb-release
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
    ```

3.  **Install ROS 2 Packages**: Install the ROS 2 `ros-foxy-desktop` (or `ros-galactic-desktop`, etc.) package. For minimal robot deployments, `ros-foxy-ros-base` might be sufficient.
    ```bash
    sudo apt update
    sudo apt install ros-foxy-desktop
    # Or for minimal installation:
    # sudo apt install ros-foxy-ros-base
    ```

4.  **Install `colcon`**: The ROS 2 build tool.
    ```bash
    sudo apt install python3-colcon-common-extensions
    ```

5.  **Environment Setup**: Source your ROS 2 installation.
    ```bash
    source /opt/ros/foxy/setup.bash
    # Add to your .bashrc for permanent sourcing:
    # echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc
    ```

### 1.3 Install Essential Dependencies

Install common development tools and ROS 2 packages required for robot operation:

```bash
sudo apt update
sudo apt install -y build-essential cmake git python3-pip
python3 -m pip install -U rosdepc
sudo rosdepc init
rosdepc update

# Install common ROS 2 packages for control, navigation, and simulation
sudo apt install -y ros-foxy-ros2-control ros-foxy-ros2-controllers ros-foxy-navigation2 ros-foxy-slam-toolbox
sudo apt install -y ros-foxy-gazebo-ros ros-foxy-gazebo-ros2-control
sudo apt install -y ros-foxy-rqt-image-view ros-foxy-rviz2
# For RealSense camera, if applicable
sudo apt install -y ros-foxy-realsense2-camera
# For other LiDARs, install their respective ROS 2 drivers
```

This robust setup ensures that your Jetson Orin is fully equipped to handle the robot's software stack and interact with its physical sensors and actuators. The next section will cover transferring your robot's code to this environment.

## S2: Sim-to-Real Code Transfer

Transferring your robot's ROS 2 workspace from your development machine to the Jetson Orin is a crucial step for physical deployment. This process ensures that the code developed and tested in simulation can be executed on the real robot, aiming for Sim-to-Real parity.

### 2.1 Prepare Your Workspace

Before transferring, ensure your local ROS 2 workspace (`colcon_ws`) is clean and properly built for the target architecture (ARM64 for Jetson). If you have built packages on an x86-64 machine, you will need to rebuild them on the Jetson.

1.  **Clean your workspace (optional, but recommended if changing architectures or for a fresh build)**:
    ```bash
    cd ~/colcon_ws
    rm -rf build install log
    ```

2.  **Ensure all necessary dependencies are in `package.xml` files**: Verify that all `build_depend`, `exec_depend`, and `depend` entries are correct for your packages.

### 2.2 Transfer the Workspace to Jetson Orin

There are several ways to transfer your `colcon_ws/src` directory to the Jetson. Using `scp` (secure copy) is a common and secure method.

From your development machine, assuming your Jetson's IP address is `192.168.1.100` and username is `jetson_user`:

```bash
scp -r ~/colcon_ws/src jetson_user@192.168.1.100:~/colcon_ws/
```

*   Replace `jetson_user` with your actual username on the Jetson.
*   Replace `192.168.1.100` with the Jetson's IP address.
*   Ensure you have `colcon_ws` directory already created on the Jetson, with a `src` subdirectory inside it (`mkdir -p ~/colcon_ws/src`).

### 2.3 Build and Source on Jetson Orin

Once the `src` directory is transferred, you need to build the workspace on the Jetson. This compiles all your ROS 2 packages specifically for the Jetson's ARM architecture.

On your Jetson Orin:

1.  **Navigate to your workspace**:
    ```bash
    cd ~/colcon_ws/
    ```

2.  **Install dependencies (if any new ones were added in `package.xml`)**:
    ```bash
    rosdepc install --from-paths src --ignore-src -r -y
    ```

3.  **Build the workspace**: Use `colcon build`. Consider selecting specific packages if only a subset has changed.
    ```bash
    colcon build --symlink-install
    ```
    *   `--symlink-install` creates symbolic links for executables and scripts, which can be useful for development as changes to scripts don't require a full rebuild.

4.  **Source the workspace**: This makes your newly built ROS 2 packages available in your environment.
    ```bash
    source install/setup.bash
    # Add to your .bashrc for permanent sourcing:
    # echo "source ~/colcon_ws/install/setup.bash" >> ~/.bashrc
    ```

After these steps, your robot's code should be successfully deployed on the Jetson Orin, ready for real-world testing. The next section will focus on integrating physical sensor drivers.

## S3: RealSense/LiDAR Driver Integration

Integrating physical sensors like LiDAR and RealSense cameras is paramount for enabling your robot to perceive its real-world environment. This section details the installation and configuration of their respective ROS 2 drivers on the Jetson Orin.

### 3.1 Intel RealSense Camera Driver

If your robot uses an Intel RealSense camera (e.g., D435i, D455), follow these steps to install and configure its ROS 2 driver.

1.  **Install the `realsense2_camera` ROS 2 package**: You may have already installed this during the initial Jetson setup. If not:
    ```bash
    sudo apt update
    sudo apt install ros-foxy-realsense2-camera
    # Or, if building from source for latest features/firmware compatibility:
    # mkdir -p ~/realsense_ws/src
    # cd ~/realsense_ws/src
    # git clone https://github.com/IntelRealSense/realsense-ros.git -b ros2-beta # Use appropriate branch
    # cd ..
    # rosdepc install --from-paths src --ignore-src -r -y
    # colcon build --symlink-install
    # source install/setup.bash
    ```

2.  **Verify Camera Detection**: Connect your RealSense camera to the Jetson via USB3. Verify it's detected:
    ```bash
    lsusb
    # Look for output similar to: Bus 001 Device 00x: ID 8086:0b07 Intel Corp. RealSense D435
    ```

3.  **Launch the RealSense Node**: Launch the camera driver to publish data:
    ```bash
    ros2 launch realsense2_camera rs_launch.py
    ```

4.  **Verify Data Publication**: Check for active topics:
    ```bash
    ros2 topic list | grep camera
    # Expected topics: /camera/depth/image_rect_raw, /camera/color/image_raw, /camera/imu, etc.
    ```

### 3.2 Physical LiDAR Driver (e.g., RPLIDAR)

If your robot uses a physical LiDAR sensor (e.g., RPLIDAR A1/A2/A3, SICK, Velodyne), follow its specific installation instructions. Here's a general example for RPLIDAR.

1.  **Install the RPLIDAR ROS 2 package**: (Assuming a community-maintained package or official driver)
    ```bash
    # Example for RPLIDAR A2/A3 (may vary)
    mkdir -p ~/lidar_ws/src
    cd ~/lidar_ws/src
    git clone https://github.com/slamtec/rplidar_ros.git -b ros2 # Use appropriate branch
    cd ..
    rosdepc install --from-paths src --ignore-src -r -y
    colcon build --symlink-install
    source install/setup.bash
    ```

2.  **Grant USB Permissions**: Ensure the Jetson user has permissions to access the LiDAR's USB port. Identify the USB device (e.g., `/dev/ttyUSB0`) and add your user to the `dialout` group.
    ```bash
    ls -l /dev/ttyUSB*
    sudo usermod -a -G dialout $USER
    # You may need to log out and back in, or reboot, for changes to take effect.
    ```

3.  **Launch the LiDAR Driver**: (Example for RPLIDAR A3)
    ```bash
    ros2 launch rplidar_ros rplidar_a3_launch.py
    ```
    *   Adjust the launch file and serial port in its configuration if necessary.

4.  **Verify Data Publication**: Check for the `/scan` topic:
    ```bash
    ros2 topic list | grep scan
    # Expected topic: /scan
    ros2 topic echo /scan
    ```

With both physical camera and LiDAR drivers integrated and publishing data, your robot is now equipped with real-world perception capabilities. The next task will focus on configuring SLAM Toolbox.

## S4: Configuring SLAM Toolbox

Simultaneous Localization and Mapping (SLAM) is a fundamental capability for autonomous robots, allowing them to build a map of an unknown environment while simultaneously localizing themselves within that map. `slam_toolbox` is a highly configurable and robust ROS 2 package for 2D SLAM.

### 4.1 Install SLAM Toolbox

If you haven't already, install `slam_toolbox` on your Jetson Orin:

```bash
sudo apt update
sudo apt install -y ros-foxy-slam-toolbox
# If building from source for specific features or compatibility:
# mkdir -p ~/slam_ws/src
# cd ~/slam_ws/src
# git clone https://github.com/ros-perception/slam_toolbox.git -b foxy-devel # Use appropriate branch
# cd ..
# rosdepc install --from-paths src --ignore-src -r -y
# colcon build --symlink-install
# source install/setup.bash
```

### 4.2 Create a Configuration File

`slam_toolbox` uses a YAML configuration file to define its parameters. Create a new file (e.g., `mapper_params_online_sync.yaml`) in your `robot_description/config/` directory (or a dedicated `robot_navigation/config/` package).

```yaml
slam_toolbox:
  ros__parameters:
    # General parameters
    use_sim_time: false         # Set to true if using simulated data (false for real robot)
    scan_topic: /scan           # Topic for incoming LiDAR scans
    odom_topic: /odom           # Topic for odometry data
    base_frame: base_link       # The robot's base frame
    odom_frame: odom            # The odometry frame
    map_frame: map              # The map frame
    mode: "sync"                # "sync" for online synchronous, "async" for asynchronous

    # Map parameters
    map_size_x: 20.0            # meters
    map_size_y: 20.0            # meters
    map_start_x: 0.0
    map_start_y: 0.0
    map_resolution: 0.05        # meters/pixel

    # Laser parameters
    max_laser_range: 5.0        # meters (adjust to your LiDAR's max effective range)
    min_laser_range: 0.1        # meters

    # Odometry correction parameters
    odom_linear_scale_correction: 1.0
    odom_angular_scale_correction: 1.0

    # Loop closure parameters (tune for your environment)
    loop_search_radius: 1.5     # meters
    loop_match_score: 0.6       # Higher is stricter
    loop_closure_enable: true

    # Visualization parameters
    enable_visualization: true
    publish_period: 1.0         # seconds
    map_pub_period: 5.0         # seconds

    # More advanced parameters can be found in the slam_toolbox documentation
```

**Key Parameters to Tune:**
*   `use_sim_time`: **Crucially, set this to `false` for your physical robot.**
*   `scan_topic`, `odom_topic`, `base_frame`, `odom_frame`, `map_frame`: Ensure these match the actual topics and TF frames your robot uses.
*   `map_resolution`, `max_laser_range`, `loop_match_score`: These heavily influence mapping quality and performance. Experiment with values suitable for your robot and environment.

### 4.3 Launch SLAM Toolbox

Integrate `slam_toolbox` into your robot's launch file (e.g., `robot_bringup.launch.py`) or create a dedicated `slam.launch.py`.

```python
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    robot_description_path = get_package_share_directory('robot_description')
    slam_toolbox_config_path = os.path.join(
        robot_description_path, 'config', 'mapper_params_online_sync.yaml'
    )

    slam_node = Node(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_toolbox_config_path
        ],
        remappings=[
            ('/scan', '/scan'), # Remap if your lidar topic is different
            ('/odom', '/odom')   # Remap if your odometry topic is different
        ]
    )

    return LaunchDescription([
        slam_node
    ])
```

After setting up the configuration and launch file, run your robot's bringup (including sensor drivers and odometry) and then launch SLAM:

```bash
colcon build --packages-select robot_description
source install/setup.bash
ros2 launch robot_description slam.launch.py # Or your main robot launch file
```

Open RViz and add a `Map` display (set topic to `/map`) and `RobotModel`. As the robot moves, you should see the map being built and the robot's pose updated in real-time. This completes the basic configuration for real-time SLAM. The next task will focus on E-Stop safety protocols.

## S5: E-Stop Safety Protocol

Implementing a robust emergency stop (E-stop) system is critical for the safe operation of any physical robot. This protocol ensures that all robot motion can be immediately halted, preventing accidents and damage. This section outlines how to integrate E-stop functionality, addressing SPEC ID `002-S4`.

### 5.1 Hardware E-Stop Integration (if applicable)

If your robot has a physical E-stop button, its integration will depend on your robot's low-level hardware. Typically, a physical E-stop directly cuts power to motors or sends a halt signal to a motor controller. Ensure that:

*   **Power Cut**: The E-stop physically disconnects motor power or signals a dedicated motor safety circuit.
*   **Latching Mechanism**: The E-stop button latches in the depressed state, requiring manual reset.
*   **Clear Indication**: The E-stop's state (active/inactive) is clearly visible via an LED or other indicator.

### 5.2 ROS 2 Software E-Stop

Beyond hardware, a software E-stop provides an additional layer of safety, allowing a remote halt command. This is usually implemented by a ROS 2 node that monitors an E-stop topic and commands zero velocity to the robot's controllers.

1.  **Create an E-Stop Monitor Node**: Develop a simple ROS 2 `Python` or `C++` node that subscribes to a dedicated E-stop topic (e.g., `/e_stop_cmd`). When a message indicating E-stop activation is received, this node should publish zero velocity commands to your robot's `cmd_vel` topic (e.g., `/diff_drive_controller/cmd_vel`).

    ```python
    # Example Python E-Stop Monitor Node (e.g., src/robot_safety/robot_safety/estop_monitor.py)
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import Bool
    from geometry_msgs.msg import Twist

    class EStopMonitor(Node):
        def __init__(self):
            super().__init__('estop_monitor')
            self.estop_active = False

            # Subscriber for E-stop commands
            self.subscription = self.create_subscription(
                Bool,
                '/e_stop_cmd',
                self.estop_callback,
                10
            )
            self.subscription # prevent unused variable warning

            # Publisher for robot velocity commands
            self.publisher = self.create_publisher(Twist, '/diff_drive_controller/cmd_vel', 10)
            self.timer = self.create_timer(0.1, self.timer_callback) # Publish zero commands frequently if estop active

            self.get_logger().info('E-Stop Monitor Node Started')

        def estop_callback(self, msg):
            self.estop_active = msg.data
            if self.estop_active:
                self.get_logger().warn('E-STOP ACTIVATED! Halting robot.')
            else:
                self.get_logger().info('E-STOP DEACTIVATED. Robot can resume motion.')

        def timer_callback(self):
            if self.estop_active:
                # Publish zero Twist command to halt the robot
                twist_msg = Twist()
                twist_msg.linear.x = 0.0
                twist_msg.angular.z = 0.0
                self.publisher.publish(twist_msg)

def main(args=None):
    rclpy.init(args=args)
    estop_monitor = EStopMonitor()
    rclpy.spin(estop_monitor)
    estop_monitor.destroy_node()
    rclpy.shutdown()

    # In your robot_description/package.xml, add:
    # <exec_depend>std_msgs</exec_depend>
    # <exec_depend>geometry_msgs</exec_depend>

    # In your robot_description/setup.py, add:
    # entry_points={
    #     'console_scripts': [
    #         'estop_monitor = robot_safety.estop_monitor:main',
    #     ],
    # },
    ```

2.  **Integrate into Launch File**: Add this node to your robot's main launch file (or a dedicated safety launch file).

    ```python
    # In your robot_bringup.launch.py
    from launch_ros.actions import Node

    # ... (other nodes)

    estop_monitor_node = Node(
        package='robot_safety',
        executable='estop_monitor',
        name='estop_monitor_node',
        output='screen',
    )

    # Add estop_monitor_node to your LaunchDescription
    return LaunchDescription([
        # ... existing nodes ...
        estop_monitor_node,
    ])
    ```

### 5.3 Testing the E-Stop

1.  **Launch Robot**: Start your robot's main launch file on the Jetson.
2.  **Move Robot**: Send a velocity command to make the robot move.
    ```bash
    ros2 topic pub /diff_drive_controller/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" --once
    ```
3.  **Activate E-Stop**: Publish `True` to the E-stop command topic:
    ```bash
    ros2 topic pub /e_stop_cmd std_msgs/msg/Bool "data: true" --once
    ```
    The robot should immediately stop. The `estop_monitor` node should log a warning.

4.  **Deactivate E-Stop**: Publish `False` to the E-stop command topic:
    ```bash
    ros2 topic pub /e_stop_cmd std_msgs/msg/Bool "data: false" --once
    ```
    The robot will remain stopped until new velocity commands are issued.

This E-stop implementation provides a critical safety mechanism for your physical robot. The next section will focus on creating modular launch files for flexible deployment.

## S6: Modular Launch Files

Modular ROS 2 launch files are essential for managing complex robot software stacks, enabling flexible deployment, easy debugging, and simplified activation/deactivation of components. This section details how to structure your launch files to achieve this, addressing SPEC ID `002-S5`.

### 6.1 Top-Level Bringup Launch File

Create a main launch file (e.g., `robot_bringup.launch.py` in your `robot_description/launch/` directory) that acts as the entry point for starting all core robot functionalities.

```python
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def launch_setup(context, *args, **kwargs):
    robot_description_path = get_package_share_directory('robot_description')

    # Parameters
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context)
    use_slam = LaunchConfiguration('use_slam').perform(context)
    use_navigation = LaunchConfiguration('use_navigation').perform(context)
    use_realsense = LaunchConfiguration('use_realsense').perform(context)
    use_lidar = LaunchConfiguration('use_lidar').perform(context)

    # Include robot_state_publisher and controllers (from Module 2)
    robot_state_publisher_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(robot_description_path, 'launch', 'robot_state_publisher.launch.py')
        ])
    )

    controllers_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(robot_description_path, 'launch', 'robot_controllers.launch.py')
        ])
    )

    # Conditional sensor launches
    sensor_nodes = []
    if use_realsense == 'true':
        sensor_nodes.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    os.path.join(get_package_share_directory('realsense2_camera'), 'launch', 'rs_launch.py')
                ])
            )
        )
    if use_lidar == 'true':
        sensor_nodes.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    os.path.join(get_package_share_directory('rplidar_ros'), 'launch', 'rplidar_a3_launch.py')
                ])
            )
        )

    # Conditional SLAM launch
    slam_node = []
    if use_slam == 'true':
        slam_config_path = os.path.join(robot_description_path, 'config', 'mapper_params_online_sync.yaml')
        slam_node.append(
            Node(
                package='slam_toolbox',
                executable='sync_slam_toolbox_node',
                name='slam_toolbox',
                output='screen',
                parameters=[slam_config_path, {'use_sim_time': use_sim_time}],
                remappings=[('/scan', '/scan'), ('/odom', '/odom')]
            )
        )

    # Conditional Navigation2 launch (placeholder)
    navigation_nodes = []
    if use_navigation == 'true':
        # Add Navigation2 launch here (e.g., using nav2_bringup)
        pass # TODO: Add navigation2 launch

    # E-stop monitor node
    estop_monitor_node = Node(
        package='robot_safety',
        executable='estop_monitor',
        name='estop_monitor_node',
        output='screen',
    )

    return [
        robot_state_publisher_launch,
        controllers_launch,
        *sensor_nodes,
        *slam_node,
        *navigation_nodes,
        estop_monitor_node,
    ]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument(
            'use_slam',
            default_value='true',
            description='Launch SLAM Toolbox if true'),
        DeclareLaunchArgument(
            'use_navigation',
            default_value='false',
            description='Launch Navigation2 if true'),
        DeclareLaunchArgument(
            'use_realsense',
            default_value='false',
            description='Launch RealSense camera driver if true'),
        DeclareLaunchArgument(
            'use_lidar',
            default_value='true',
            description='Launch LiDAR driver if true'),
        OpaqueFunction(function=launch_setup)
    ])
```

### 6.2 Component-Specific Launch Files

Break down complex components into their own launch files. Examples:

*   **`robot_state_publisher.launch.py`** (for `robot_state_publisher` and `joint_state_broadcaster`)
*   **`robot_controllers.launch.py`** (for `diff_drive_controller` and other controllers)
*   **`sensors.launch.py`** (to encapsulate all sensor drivers, which can then be conditionally included)
*   **`slam.launch.py`** (for `slam_toolbox`)
*   **`navigation.launch.py`** (for `navigation2` stack)

Each of these can be called by the top-level `robot_bringup.launch.py` using `IncludeLaunchDescription`.

### 6.3 Conditional Launching and Parameters

The example `robot_bringup.launch.py` demonstrates:
*   **`DeclareLaunchArgument`**: Defines parameters that can be passed at launch time (e.g., `use_sim_time`, `use_slam`).
*   **`LaunchConfiguration`**: Retrieves the value of launch arguments.
*   **Conditional Logic**: Using Python `if` statements (within `OpaqueFunction`) to include/exclude nodes or launch files based on parameter values (e.g., only launch SLAM if `use_slam` is `true`).

### 6.4 Switching Between Simulation and Real Robot

The `use_sim_time` launch argument is crucial for Sim-to-Real parity. When `use_sim_time` is `true`, ROS 2 nodes will synchronize their clocks with the simulation time (published by Gazebo). For the physical robot, `use_sim_time` should always be `false`.

To launch your robot in the real world with SLAM and LiDAR:

```bash
ros2 launch robot_description robot_bringup.launch.py use_sim_time:=false use_slam:=true use_lidar:=true use_realsense:=false
```

This modular approach ensures flexibility and scalability for your robot's deployment. The next task will focus on real-time SLAM testing.

## S7: Real-time SLAM Testing

After configuring `slam_toolbox` and setting up modular launch files, it's crucial to test the real-time Simultaneous Localization and Mapping (SLAM) capabilities on your physical robot. This step directly addresses SPEC ID `002-S3` by verifying map consistency and localization accuracy in a real environment.

### 7.1 Prepare the Environment

Choose an indoor environment with some distinct features (e.g., walls, furniture) but avoid overly reflective surfaces or areas with dynamic obstacles initially. Ensure good lighting conditions for camera-based systems if applicable.

### 7.2 Launch Robot and SLAM

1.  **Start your robot's main bringup launch file on the Jetson Orin**:
    ```bash
    ros2 launch robot_description robot_bringup.launch.py use_sim_time:=false use_slam:=true use_lidar:=true
    # Adjust 'use_lidar' and 'use_realsense' based on your sensor setup
    ```

2.  **Launch RViz on a remote workstation (if available)**: If you have a separate development machine, launch RViz there to visualize the map and robot pose. Ensure your `ROS_DOMAIN_ID` and network configuration allow communication.
    ```bash
    ros2 run rviz2 rviz2 -d src/robot_description/rviz/robot_config.rviz
    ```

    *   In RViz, add `Map` (topic: `/map`) and `RobotModel` displays. Set `Fixed Frame` to `map`.

### 7.3 Execute SLAM Test Procedure

1.  **Initial Scan**: Carefully move the robot slowly in a small area to allow `slam_toolbox` to build an initial portion of the map. Observe the `/map` topic in RViz.

2.  **Explore the Environment**: Systematically drive the robot through the environment, ensuring the LiDAR (and camera, if used for visual odometry) covers all areas. Move at a moderate, consistent speed.

3.  **Verify Map Consistency**: As the robot explores, pay attention to the map being generated in RViz:
    *   **No Ghosting/Double Mapping**: Ensure the map does not show duplicated features when the robot revisits an area.
    *   **Loop Closure**: Observe if `slam_toolbox` detects and corrects loops (when the robot returns to a previously mapped location), resulting in a more consistent and accurate map.
    *   **Sharp Features**: Walls and distinct objects should appear sharp and well-defined in the map.

4.  **Verify Localization Accuracy**: To test SPEC ID `002-S3` (drift less than 5cm over 5 meters of travel):
    *   **Path Traversal**: Drive the robot along a precisely measured 5-meter straight line and then execute a 90-degree turn.
    *   **Measure Actual vs. Mapped**: After traversing, compare the robot's reported pose in RViz against its actual physical position and orientation. Manually measure any discrepancy.
    *   **Repeatability**: Repeat this process multiple times to ensure consistency.

5.  **Test Real-time Performance (SPEC ID `002-S3` acceptance criteria)**:
    *   Monitor CPU/GPU usage on the Jetson during SLAM operation (`jtop` or `htop`).
    *   Check `ros2 topic hz /map` and `ros2 topic hz /tf` to ensure map and pose updates are meeting the required frequencies (e.g., map updates at least 5 Hz, pose updates at least 30 Hz).

Successful real-time SLAM testing confirms your robot's ability to autonomously map and localize in real-world conditions. The next task will involve verifying Sim-to-Real parity.

## S8: Verifying Sim-to-Real Parity

Achieving Sim-to-Real parity (SPEC ID `002-S2`) is fundamental for reliably transferring algorithms developed in simulation to a physical robot. This section outlines a methodology to quantitatively compare the behavior of your simulated robot in Gazebo with that of your physical robot under similar conditions.

### 8.1 Establish a Controlled Test Environment

*   **Physical Environment**: Select a clear, flat, and unobstructed area in the real world that can be precisely measured. Mark a starting point, a 2-meter straight line, and a clear 90-degree turn location.
*   **Simulated Environment**: Ensure your Gazebo world accurately reflects the physical test environment (e.g., a flat ground plane). The robot model and its physics parameters should be well-calibrated.

### 8.2 Standardized Test Procedure

1.  **Reset Robot Pose**: Ensure both the simulated and physical robots start from identical initial poses (position and orientation) at the marked starting point.

2.  **Execute Identical Control Commands**: Use precise ROS 2 `Twist` messages to command both robots to perform a specific movement sequence. For example:
    *   **2-meter straight line**: Publish `linear.x = 0.5 m/s, angular.z = 0.0 rad/s` for 4 seconds.
    *   **90-degree turn**: Publish `linear.x = 0.0 m/s, angular.z = 0.785 rad/s` for 2 seconds.

    ```bash
    # Example command for 0.5 m/s linear for 4 seconds
    ros2 topic pub /diff_drive_controller/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" --duration 4
    # Example command for 0.785 rad/s angular for 2 seconds
    ros2 topic pub /diff_drive_controller/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.785}}" --duration 2
    ```

3.  **Record Data**: For both simulated and physical runs, record the following:
    *   **Odometry Data**: Subscribe to `/odom` topic and save `geometry_msgs/msg/Odometry` messages.
    *   **Joint States**: Subscribe to `/joint_states` to get individual wheel velocities.
    *   **Sensor Data**: (Optional, but recommended) Record LiDAR `/scan` and camera `/camera/image_raw` topics.

    Use `ros2 bag record -a` to record all topics for later analysis.

### 8.3 Analyze and Compare Data

1.  **Odometry Comparison (SPEC ID `002-S2.2`)**:
    *   After the robot completes the path, compare the final (x, y, yaw) pose from the `/odom` topic for both physical and simulated runs.
    *   Calculate the percentage deviation for linear distance and angular rotation. The deviation should be within the specified 10% margin.

2.  **Velocity Profile Comparison (SPEC ID `002-S2.1`)**:
    *   Plot the linear and angular velocities (from `/odom` or calculated from `joint_states`) over time for both runs.
    *   Visually inspect and quantitatively compare the peak velocities, acceleration/deceleration ramps, and steady-state velocities. Deviations should be within ±5%.

3.  **Sensor Data Qualitative Comparison (SPEC ID `002-S2.3`)**:
    *   **LiDAR**: Compare `/scan` data from both environments in RViz. The point clouds should show similar patterns for identical obstacles.
    *   **Camera**: Visually inspect recorded camera feeds. Objects, lighting, and textures should appear reasonably similar, allowing vision algorithms to function across both.

### 8.4 Iterate and Refine

If significant discrepancies are found, iterate on:
*   **URDF Parameters**: Adjust mass, inertia, friction coefficients in your robot's URDF and Gazebo plugins to better match physical properties.
*   **Controller Gains**: Tune PID gains of your `diff_drive_controller` (or other controllers) to optimize physical robot responsiveness.
*   **Sensor Noise**: Add realistic noise models to your Gazebo sensor plugins to mimic real-world sensor imperfections.

This rigorous comparison process ensures that your digital twin is an accurate representation of your physical robot, allowing for effective Sim-to-Real transfer. The final task will consolidate all procedures into a deployment guide.