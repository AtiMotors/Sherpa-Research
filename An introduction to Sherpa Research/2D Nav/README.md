# 2D Navigation - Sherpa RP

## Overview
This directory contains packages and documentation for 2D navigation and SLAM (Simultaneous Localization and Mapping) capabilities of the Sherpa RP platform using ROS2 Nav2 stack.

## Navigation Stack Architecture

### Core Components
- **Nav2 Stack**: Complete autonomous navigation solution
- **Cartographer**: SLAM algorithm for map building
- **AMCL**: Adaptive Monte Carlo Localization
- **Local/Global Planners**: Path planning algorithms
- **Behavior Trees**: Decision-making framework

### Sensors Used
- **SlamTec RPLidar C1**: Primary 2D LiDAR sensor
- **Intel RealSense D435i**: Supplementary depth data
- **Motor Encoders**: Odometry feedback via CAN bus

## Quick Start Guide

### Prerequisites
```bash
# Ensure ROS2 Humble is installed and sourced
source /opt/ros/humble/setup.bash

# Install Nav2 dependencies
sudo apt install ros-humble-nav2-bringup
sudo apt install ros-humble-navigation2
sudo apt install ros-humble-nav2-map-server
sudo apt install ros-humble-nav2-graceful-controller
```

### Building the Workspace
```bash
# Navigate to your ROS2 workspace
cd ~/ros2_ws

# Build packages
colcon build --packages-select edubot_bringup localization_server

# Source the workspace
source install/setup.bash
```

## Mapping Process

### Step 1: Launch Mapping Node
```bash
# Terminal 1: Start the mapping launch file
ros2 launch edubot_bringup mapping_launch.launch.py
```

### Step 2: Manual Control for Exploration
```bash
# Terminal 2: Start joystick control
ros2 run joy joy_node

# Terminal 3: Enable teleop
ros2 launch edubot_bringup teleop_launch_launch.py
```

### Step 3: Save the Map
```bash
# Create maps directory if it doesn't exist
mkdir -p ~/ros2_ws/src/localization_server/maps/

# Save the generated map
ros2 run nav2_map_server map_saver_cli -f ~/ros2_ws/src/localization_server/maps/my_map
```

### Map File Structure
After mapping, you'll have:
- `my_map.pgm`: Binary occupancy grid image
- `my_map.yaml`: Map metadata and parameters

## Localization Setup

### Configure Map in Launch File
Edit `~/ros2_ws/src/localization_server/launch/localization.launch.py`:

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    map_file = os.path.join(
        get_package_share_directory('localization_server'),
        'maps', 'my_map.yaml'
    )
    
    amcl_yaml = os.path.join(
        get_package_share_directory('localization_server'),
        'config', 'amcl_config.yaml'
    )
    
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            {'use_sim_time': False},
            {'yaml_filename': map_file}
        ]
    )
    
    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[amcl_yaml]
    )
    
    lifecycle_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[
            {'use_sim_time': False},
            {'autostart': True},
            {'node_names': ['map_server', 'amcl']}
        ]
    )
    
    ld = LaunchDescription()
    ld.add_action(map_server_node)
    ld.add_action(amcl_node)
    ld.add_action(lifecycle_node)
    return ld
```

## Navigation Configuration

### AMCL Parameters (`config/amcl_config.yaml`)
```yaml
amcl:
  ros__parameters:
    use_sim_time: false
    alpha1: 0.2
    alpha2: 0.2
    alpha3: 0.2
    alpha4: 0.2
    alpha5: 0.2
    base_frame_id: "base_link"
    beam_skip_distance: 0.5
    beam_skip_error_threshold: 0.9
    beam_skip_threshold: 0.3
    do_beamskip: false
    global_frame_id: "map"
    lambda_short: 0.1
    laser_likelihood_max_dist: 2.0
    laser_max_range: 100.0
    laser_min_range: -1.0
    laser_model_type: "likelihood_field"
    max_beams: 60
    max_particles: 2000
    min_particles: 500
    odom_frame_id: "odom"
    pf_err: 0.05
    pf_z: 0.99
    recovery_alpha_fast: 0.0
    recovery_alpha_slow: 0.0
    resample_interval: 1
    robot_model_type: "differential"
    save_pose_rate: 0.5
    sigma_hit: 0.2
    tf_broadcast: true
    transform_tolerance: 1.0
    update_min_a: 0.2
    update_min_d: 0.25
    z_hit: 0.5
    z_max: 0.05
    z_rand: 0.5
    z_short: 0.05
```

## Launch Files

### Mapping Launch (`launch/mapping_launch.launch.py`)
```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='slam_toolbox',
            executable='sync_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[
                {'use_sim_time': False},
                {'slam_toolbox/map_file_name': '/tmp/my_map'},
                {'slam_toolbox/map_start_at_dock': True}
            ]
        ),
        
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': 'robot_description'}]
        )
    ])
```

## Troubleshooting

### Common Issues

1. **LiDAR Not Detected**
   ```bash
   # Check LiDAR connection
   ls /dev/ttyUSB*
   
   # Verify LiDAR permissions
   sudo chmod 777 /dev/ttyUSB0
   ```

2. **Map Quality Issues**
   - Ensure adequate lighting (minimum 50 Lux)
   - Move slowly during mapping for better scan quality
   - Avoid reflective surfaces and glass

3. **Localization Failures**
   ```bash
   # Check TF tree
   ros2 run tf2_tools view_frames.py
   
   # Monitor AMCL pose estimates
   ros2 topic echo /amcl_pose
   ```

4. **Navigation Stack Not Starting**
   ```bash
   # Check all required nodes
   ros2 node list
   
   # Verify parameter loading
   ros2 param list
   ```

### Performance Optimization

- **Scan Frequency**: Adjust LiDAR scan rate for environment complexity
- **Particle Count**: Increase for large, complex environments
- **Update Thresholds**: Fine-tune for movement sensitivity

## RViz Visualization

Launch RViz to visualize mapping and navigation:
```bash
rviz2 -d ~/ros2_ws/src/edubot_bringup/rviz/nav2_config.rviz
```

### Key RViz Displays
- **LaserScan**: LiDAR data visualization
- **Map**: Occupancy grid display
- **RobotModel**: 3D robot representation
- **PoseArray**: AMCL particle cloud
- **Path**: Planned trajectory

## Integration with Hardware

### CAN Bus Integration
The navigation system interfaces with Sherpa RP's motor controllers via CAN bus:
- **Topic**: `/cmd_vel` (geometry_msgs/Twist)
- **Node**: `robot_control_node`
- **Protocol**: CAN 2.0b at 1Mbps
- **Hardware**: Twara CAN USB transceiver

### Safety Integration
- **EPO System**: Emergency stop functionality
- **Collision Avoidance**: Real-time obstacle detection
- **Speed Limits**: Configurable maximum velocities

## Additional Resources

- [Nav2 Documentation](https://navigation.ros.org/)
- [Cartographer Documentation](https://google-cartographer.readthedocs.io/)
- [ROS2 Navigation Tutorials](https://docs.ros.org/en/humble/Tutorials/Advanced/Navigation/Navigation-Main.html)
