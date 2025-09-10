# ROS2 Implementation - Sherpa RP

## Overview
This directory contains ROS2 Humble Hawksbill implementation packages, configurations, and documentation for the Sherpa RP platform. The system runs on Ubuntu 22.04 within Docker containers for hardware agnostic deployment.

## System Architecture

### Docker-Based Deployment
The Sherpa RP uses a containerized architecture for maximum portability and isolation:
- **Host OS**: Ubuntu 20.04 (SBC native)
- **Container OS**: Ubuntu 22.04 LTS
- **ROS2 Distribution**: Humble Hawksbill
- **Containerization**: Docker with Docker Compose

### Core ROS2 Nodes

#### 1. Navigation Stack
- **Package**: `nav2_bringup`
- **Purpose**: Autonomous navigation and path planning
- **Dependencies**: Nav2, Cartographer, AMCL
- **Key Topics**: `/cmd_vel`, `/goal_pose`, `/map`

#### 2. Mapping Stack  
- **Package**: `slam_toolbox` / `cartographer_ros`
- **Purpose**: SLAM (Simultaneous Localization and Mapping)
- **Sensors**: RPLidar C1, Intel RealSense D435i
- **Output**: Occupancy grid maps

#### 3. Robot Control Node
- **Package**: `edubot_control`
- **Purpose**: Convert twist commands to differential drive
- **Interface**: CAN bus communication with motor drivers
- **Safety**: EPO integration and velocity limiting

#### 4. Motor Control Node
- **Package**: `twara_motor_driver`
- **Purpose**: Interface with BLDC motor controllers
- **Protocol**: CAN 2.0b at 1Mbps
- **Features**: Speed, torque, and position control

#### 5. Sensor Nodes
- **LiDAR**: `rplidar_ros` - RPLidar C1 interface
- **Camera**: `realsense2_camera` - Intel RealSense D435i
- **IMU**: Integrated with RealSense D435i
- **Joystick**: `joy` - Sony DualShock 4 controller

## Installation and Setup

### System Requirements
```bash
# Operating System
Ubuntu 22.04 LTS (in Docker)
ROS2 Humble Hawksbill
Docker Engine 20.10+
Docker Compose 2.0+

# Hardware Requirements
Minimum 4GB RAM (8GB recommended)
16GB storage space
USB 3.0 ports for sensors
CAN interface for motor control
```

### Docker Environment Setup

#### For Raspberry Pi 5
```bash
# Install Docker
sudo apt install docker.io
sudo usermod -aG docker $USER

# Navigate to Docker directory
cd ~/Sherpa_RP/docker/ros2_humble_pi/

# Build Docker image
docker build -t ros2_humble_pi .

# Update compose.yaml with correct paths
# Edit volumes section to point to your ros2_ws directory
volumes:
  - ~/Sherpa_RP/ros2_ws:/home/ros/ros2_ws

# Launch container
docker compose up -d
```

#### For NVIDIA Jetson Orin Nano
```bash
# Flash Jetpack 5.1.4 first using SDK Manager

# Install dependencies
sudo apt update && sudo apt upgrade
sudo apt install can-utils openssl libssl-dev build-essential bc git

# Install Docker
sudo apt install docker.io
sudo usermod -aG docker $USER
reboot

# Build and run container
cd ~/ati.Sherpa_RP/docker/ros2_humble_orin_nano/
docker build -t ros2_humble_orin_nano .
docker compose up -d
docker exec -it SherpaRP /bin/bash
```

#### For Development PC (AMD64)
```bash
# Install Docker
curl -fsSL https://get.docker.com -o get-docker.sh
sudo sh ./get-docker.sh
sudo usermod -aG docker $USER
reboot

# Build development environment
cd ~/Sherpa_RP/docker/ros2_humble_amd/
docker build -t ros2_humble_amd .
```

### ROS2 Workspace Setup

#### Inside Docker Container
```bash
# Navigate to workspace
cd ~/ros2_ws

# Install dependencies
sudo apt update && sudo apt upgrade
sudo apt install ros-humble-console-bridge-vendor
sudo rosdep init && rosdep update
rosdep install --from-paths src --ignore-src -r -y

# Build workspace
colcon build

# Source environment
source install/setup.bash
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
```

### Package Structure
```
ros2_ws/
├── src/
│   ├── edubot_bringup/          # Main launch package
│   ├── edubot_control/          # Robot control logic
│   ├── edubot_description/      # URDF robot models
│   ├── localization_server/     # Mapping and localization
│   ├── twara_motor_driver/      # CAN motor interface
│   ├── sensor_drivers/          # Custom sensor packages
│   └── teleop_packages/         # Manual control
├── install/                     # Built packages
├── build/                       # Build artifacts
└── log/                        # Build and runtime logs
```

## Launch Configurations

### Complete System Launch
```bash
# Launch all robot systems
ros2 launch edubot_bringup robot_bringup.launch.py
```

### Individual System Components

#### Navigation System
```bash
# Autonomous navigation
ros2 launch edubot_bringup nav2_launch.launch.py

# Mapping mode
ros2 launch edubot_bringup mapping_launch.launch.py
```

#### Manual Control
```bash
# Joystick control
ros2 run joy joy_node
ros2 launch edubot_bringup teleop_launch.launch.py
```

#### Sensor Drivers
```bash
# LiDAR only
ros2 launch rplidar_ros rplidar_c1_launch.py

# RealSense camera
ros2 launch realsense2_camera rs_launch.py

# All sensors
ros2 launch edubot_bringup sensors_launch.py
```

## Configuration Files

### Robot Description (URDF)
Located in `edubot_description/urdf/sherpa_rp.urdf.xacro`:
```xml
<?xml version="1.0"?>
<robot name="sherpa_rp" xmlns:xacro="http://www.ros.org/wiki/xacro">
  <!-- Robot dimensions and properties -->
  <xacro:property name="wheel_radius" value="0.1" />
  <xacro:property name="wheel_width" value="0.05" />
  <xacro:property name="base_width" value="0.6" />
  <xacro:property name="base_length" value="0.8" />
  
  <!-- Include sensor definitions -->
  <xacro:include filename="$(find edubot_description)/urdf/sensors.xacro" />
  <xacro:include filename="$(find edubot_description)/urdf/wheels.xacro" />
  
  <!-- Base link definition -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="${base_length} ${base_width} 0.2"/>
      </geometry>
      <material name="blue"/>
    </visual>
    <collision>
      <geometry>
        <box size="${base_length} ${base_width} 0.2"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="44.0"/>
      <inertia ixx="1.0" ixy="0.0" ixz="0.0" 
               iyy="1.0" iyz="0.0" izz="1.0"/>
    </inertial>
  </link>
  
  <!-- Sensor mounts and joints -->
  <xacro:rplidar_c1 parent="base_link"/>
  <xacro:realsense_d435i parent="base_link"/>
</robot>
```

### Navigation Parameters
`config/nav2_params.yaml`:
```yaml
bt_navigator:
  ros__parameters:
    use_sim_time: False
    global_frame: map
    robot_base_frame: base_link
    odom_topic: /odom
    bt_loop_duration: 10
    default_server_timeout: 20
    
controller_server:
  ros__parameters:
    use_sim_time: False
    controller_frequency: 20.0
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 0.5
    min_theta_velocity_threshold: 0.001
    progress_checker_plugin: "progress_checker"
    goal_checker_plugins: ["general_goal_checker"]
    controller_plugins: ["FollowPath"]
    
    FollowPath:
      plugin: "dwb_core::DWBLocalPlanner"
      debug_trajectory_details: True
      min_vel_x: 0.0
      min_vel_y: 0.0
      max_vel_x: 0.7
      max_vel_y: 0.0
      max_vel_theta: 2.0
      min_speed_xy: 0.0
      max_speed_xy: 0.7
      min_speed_theta: 0.0
      acc_lim_x: 1.0
      acc_lim_y: 0.0
      acc_lim_theta: 2.0
      decel_lim_x: -1.0
      decel_lim_y: 0.0
      decel_lim_theta: -2.0

global_costmap:
  global_costmap:
    ros__parameters:
      footprint: "[[-0.3, -0.3], [-0.3, 0.3], [0.3, 0.3], [0.3, -0.3]]"
      resolution: 0.05
      track_unknown_space: true
      plugins: ["static_layer", "obstacle_layer", "inflation_layer"]
      
local_costmap:
  local_costmap:
    ros__parameters:
      footprint: "[[-0.3, -0.3], [-0.3, 0.3], [0.3, 0.3], [0.3, -0.3]]"
      resolution: 0.05
      rolling_window: true
      width: 3
      height: 3
      plugins: ["obstacle_layer", "inflation_layer"]
```

## Development Workflow

### Building Packages
```bash
# Build all packages
cd ~/ros2_ws
colcon build

# Build specific package
colcon build --packages-select edubot_bringup

# Build with debug symbols
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug
```

### Testing and Debugging
```bash
# List active nodes
ros2 node list

# Check node information
ros2 node info /robot_control_node

# Monitor topics
ros2 topic list
ros2 topic echo /cmd_vel
ros2 topic hz /scan

# Service calls
ros2 service list
ros2 service call /emergency_stop std_srvs/srv/Trigger

# Parameter management
ros2 param list
ros2 param get /controller_server use_sim_time
ros2 param set /controller_server max_vel_x 0.5
```

### Logging and Diagnostics
```bash
# View logs
ros2 log info

# Enable debug logging
ros2 run rcl_logging_demo logging_demo_main --ros-args --log-level DEBUG

# Monitor system resources
ros2 run system_monitor system_monitor_node
```

## CAN Bus Integration

### Hardware Interface
The Sherpa RP uses Twara CAN USB transceiver for motor communication:
```bash
# Setup CAN interface
sudo modprobe can
sudo modprobe can_raw
sudo ip link set can0 type can bitrate 1000000
sudo ip link set up can0

# Test CAN communication
cansend can0 123#DEADBEEF
candump can0
```

### ROS2 CAN Driver
```python
# Example CAN motor controller node
import rclpy
from rclpy.node import Node
import can
from geometry_msgs.msg import Twist

class MotorController(Node):
    def __init__(self):
        super().__init__('motor_controller')
        self.bus = can.interface.Bus(channel='can0', bustype='socketcan')
        self.subscription = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)
    
    def cmd_vel_callback(self, msg):
        # Convert twist to motor commands
        left_speed = msg.linear.x - msg.angular.z * 0.3
        right_speed = msg.linear.x + msg.angular.z * 0.3
        
        # Send CAN messages
        left_msg = can.Message(arbitration_id=0x141, 
                              data=self.encode_speed(left_speed))
        right_msg = can.Message(arbitration_id=0x142, 
                               data=self.encode_speed(right_speed))
        
        self.bus.send(left_msg)
        self.bus.send(right_msg)
```

## Performance Optimization

### System Tuning
```bash
# Set thread priorities for real-time performance
sudo echo 'kernel.sched_rt_period_us=1000000' >> /etc/sysctl.conf
sudo echo 'kernel.sched_rt_runtime_us=950000' >> /etc/sysctl.conf

# Optimize network for ROS2 communication
export RMW_IMPLEMENTATION=rmw_cyclonedx_cpp
export CYCLONEDX_URI='<CycloneDX><Domain><General><NetworkInterfaceAddress>wlan0</NetworkInterfaceAddress></General></Domain></CycloneDX>'
```

### Memory Management
```bash
# Monitor memory usage
ros2 run resource_monitor memory_monitor

# Optimize Docker memory
docker run --memory=4g --memory-swap=4g ros2_humble_pi
```

## Troubleshooting

### Common Issues

1. **ROS2 Not Sourced**
```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
```

2. **Docker Permission Issues**
```bash
sudo usermod -aG docker $USER
# Log out and back in
```

3. **CAN Bus Not Available**
```bash
# Check kernel modules
lsmod | grep can
sudo modprobe can can_raw

# Verify interface
ip link show can0
```

4. **Node Communication Issues**
```bash
# Check ROS2 daemon
ros2 daemon stop
ros2 daemon start

# Verify network discovery
ros2 topic list
ros2 node list
```

## Migration from ROS1

For legacy ROS1 support, see the `/ROS1` directory. Key differences:
- **Launch Files**: XML → Python
- **Parameters**: Parameter server → Node parameters
- **Build System**: catkin → colcon
- **Communication**: Topics remain similar, services and actions updated

## Integration Examples

### Custom Service Example
```python
# emergency_stop_service.py
from std_srvs.srv import Trigger

class EmergencyStopService(Node):
    def __init__(self):
        super().__init__('emergency_stop_service')
        self.srv = self.create_service(
            Trigger, 'emergency_stop', self.emergency_stop_callback)
    
    def emergency_stop_callback(self, request, response):
        # Trigger EPO sequence
        self.stop_all_motors()
        response.success = True
        response.message = 'Emergency stop activated'
        return response
```

### Custom Action Example
```python
# navigate_to_goal_action.py
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose

class NavigationController(Node):
    def __init__(self):
        super().__init__('navigation_controller')
        self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
    
    def send_goal(self, x, y, theta):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.pose.position.x = float(x)
        goal_msg.pose.pose.position.y = float(y)
        goal_msg.pose.header.frame_id = 'map'
        
        self.action_client.wait_for_server()
        return self.action_client.send_goal_async(goal_msg)
```

## References and Documentation

### Official ROS2 Resources
- [ROS2 Humble Documentation](https://docs.ros.org/en/humble/)
- [Nav2 Documentation](https://navigation.ros.org/)
- [ROS2 Tutorials](https://docs.ros.org/en/humble/Tutorials.html)

### Hardware-Specific Documentation
- [Raspberry Pi 5 Setup](../companion-computer/raspberry-pi/README.md)
- [NVIDIA Jetson Setup](../companion-computer/jetson/README.md)
- [CAN Bus Configuration](../controls/README.md)

### Sherpa RP Specific
- [Hardware Architecture](../CAD_Files/README.md)
- [Sensor Integration](../sensors/README.md)
- [Safety Systems](../controls/README.md)
