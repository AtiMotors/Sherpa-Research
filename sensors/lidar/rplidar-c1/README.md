# LiDAR Integration - Sherpa RP

## Overview
This directory contains packages, drivers, and documentation for LiDAR sensor integration on the Sherpa RP platform. The primary LiDAR sensor is the SlamTec RPLidar C1, which provides 2D laser scanning for navigation, mapping, and obstacle detection.

## Hardware Specifications

### SlamTec RPLidar C1
- **Model**: RPLidar C1
- **Technology**: Triangle ranging technology
- **Range**: 0.1m - 12m
- **Resolution**: 360° scanning
- **Sample Rate**: Up to 8000 Hz
- **Angular Resolution**: 0.5° - 2°
- **Rotation Speed**: 300-600 RPM (adjustable)
- **Interface**: USB 2.0 with TTL UART
- **Power Consumption**: ~2W
- **Operating Temperature**: -10°C to +50°C
- **Laser Class**: Class 1 (eye-safe)

### Physical Mounting
- **Location**: Top of robot chassis
- **Height**: Approximately 0.3m from ground
- **Mounting**: Standard 2020 aluminum extrusion slots
- **Protection**: Dust cover and impact protection
- **Orientation**: 360° unobstructed view

## Software Architecture

### ROS2 Integration
```
┌─────────────────┐    ┌──────────────────┐    ┌─────────────────┐
│   RPLidar C1    │───▶│  rplidar_ros     │───▶│  Navigation     │
│   (Hardware)    │    │     (Driver)     │    │    Stack        │
└─────────────────┘    └──────────────────┘    └─────────────────┘
```

### Key ROS2 Topics
- **Output**: `/scan` (sensor_msgs/LaserScan)
- **Status**: `/rplidar_health` (std_msgs/String)
- **Control**: `/start_scan`, `/stop_scan` (std_srvs/Empty)

### Message Format
```
sensor_msgs/LaserScan:
  header:
    stamp: # Timestamp
    frame_id: "laser_frame"
  angle_min: -3.14159  # Start angle (radians)
  angle_max: 3.14159   # End angle (radians)
  angle_increment: 0.0174533  # Angular resolution (radians)
  time_increment: 0.0  # Time between measurements (seconds)
  scan_time: 0.1       # Time for complete scan (seconds)
  range_min: 0.1       # Minimum range (meters)
  range_max: 12.0      # Maximum range (meters)
  ranges: []           # Range measurements (meters)
  intensities: []      # Signal intensity values
```

## Installation and Setup

### Hardware Connection
1. **Physical Connection**:
   - Connect RPLidar C1 to USB port on SBC
   - Ensure stable mounting on robot chassis
   - Verify unobstructed 360° rotation

2. **Power Requirements**:
   - USB bus power (5V, <500mA)
   - No external power supply needed

3. **Verification**:
   ```bash
   # Check USB connection
   lsusb | grep CP2102
   
   # Check device node
   ls /dev/ttyUSB*
   
   # Set permissions
   sudo chmod 666 /dev/ttyUSB0
   ```

### Software Installation
```bash
# Install RPLidar ROS2 driver
sudo apt update
sudo apt install ros-humble-rplidar-ros

# Or build from source
cd ~/ros2_ws/src
git clone https://github.com/Slamtec/rplidar_ros.git
cd ../
colcon build --packages-select rplidar_ros
```

### Driver Configuration
Create configuration file `config/rplidar_params.yaml`:
```yaml
rplidar_composition:
  ros__parameters:
    serial_port: '/dev/ttyUSB0'
    serial_baudrate: 256000
    frame_id: 'laser_frame'
    inverted: false
    angle_compensate: true
    scan_mode: 'Standard'  # Standard, Express, Boost
    auto_standby: true
    max_distance: 12.0
    min_distance: 0.1
    frequency: 10.0  # Scan frequency (Hz)
```

## Launch Configuration

### Basic LiDAR Launch
Create `launch/rplidar_launch.py`:
```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'serial_port',
            default_value='/dev/ttyUSB0',
            description='Serial port for RPLidar'
        ),
        
        DeclareLaunchArgument(
            'frame_id',
            default_value='laser_frame',
            description='Frame ID for laser scans'
        ),
        
        Node(
            package='rplidar_ros',
            executable='rplidar_composition',
            name='rplidar_node',
            output='screen',
            parameters=[{
                'serial_port': LaunchConfiguration('serial_port'),
                'serial_baudrate': 256000,
                'frame_id': LaunchConfiguration('frame_id'),
                'inverted': False,
                'angle_compensate': True,
                'scan_mode': 'Standard'
            }]
        ),
        
        # Static transform publisher for sensor frame
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='laser_frame_publisher',
            arguments=[
                '0', '0', '0.3',  # x, y, z offset
                '0', '0', '0',    # roll, pitch, yaw
                'base_link',      # parent frame
                'laser_frame'     # child frame
            ]
        )
    ])
```

### Integrated System Launch
```python
# launch/sensors_launch.py - Complete sensor suite
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    return LaunchDescription([
        # RPLidar
        Node(
            package='rplidar_ros',
            executable='rplidar_composition',
            name='rplidar_node',
            parameters=['/path/to/rplidar_params.yaml']
        ),
        
        # RealSense Camera (for additional depth data)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                get_package_share_directory('realsense2_camera'),
                '/launch/rs_launch.py'
            ])
        ),
        
        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_description}]
        )
    ])
```

## Calibration and Testing

### Sensor Verification
```bash
# Launch LiDAR driver
ros2 launch rplidar_ros rplidar_launch.py

# Verify scan data
ros2 topic echo /scan

# Check scan frequency
ros2 topic hz /scan

# Monitor for errors
ros2 topic echo /rplidar_health
```

### Data Quality Assessment
```bash
# Visualize in RViz
rviz2 -d ~/ros2_ws/src/sherpa_config/rviz/lidar_view.rviz

# Save scan data for analysis
ros2 bag record /scan

# Analyze scan statistics
ros2 run laser_scan_tools scan_analyzer /scan
```

### Calibration Procedure
1. **Physical Alignment**:
   - Ensure LiDAR is level and stable
   - Verify mounting screws are tight
   - Check for any obstructions

2. **Software Calibration**:
   ```bash
   # Static environment scan
   ros2 service call /start_scan std_srvs/srv/Empty
   
   # Record calibration data
   ros2 bag record -o calibration_data /scan /tf_static
   
   # Analyze and adjust parameters
   ros2 param set /rplidar_node angle_compensate true
   ```

## Navigation Integration

### SLAM Integration
The LiDAR integrates with SLAM algorithms for mapping:
```yaml
# slam_toolbox_params.yaml
slam_toolbox:
  ros__parameters:
    # Sensor configuration
    laser_topic: '/scan'
    base_frame: 'base_link'
    odom_frame: 'odom'
    map_frame: 'map'
    
    # SLAM parameters
    resolution: 0.05
    max_laser_range: 12.0
    minimum_time_interval: 0.5
    transform_publish_period: 0.02
    
    # Loop closure
    loop_search_maximum_distance: 3.0
    do_loop_closing: true
    loop_match_minimum_response_coarse: 0.35
```

### Navigation Stack Integration
```yaml
# local_costmap_params.yaml for obstacle avoidance
local_costmap:
  local_costmap:
    ros__parameters:
      global_frame: 'odom'
      robot_base_frame: 'base_link'
      
      plugins: ['obstacle_layer', 'inflation_layer']
      
      obstacle_layer:
        plugin: 'nav2_costmap_2d::ObstacleLayer'
        enabled: True
        observation_sources: 'scan'
        scan:
          topic: '/scan'
          max_obstacle_height: 2.0
          clearing: True
          marking: True
          data_type: 'LaserScan'
          raytrace_max_range: 10.0
          raytrace_min_range: 0.0
          obstacle_max_range: 8.0
          obstacle_min_range: 0.0
```

## Advanced Features

### Multi-LiDAR Setup
For enhanced coverage or redundancy:
```python
# launch/multi_lidar_launch.py
def generate_launch_description():
    return LaunchDescription([
        # Primary LiDAR (front)
        Node(
            package='rplidar_ros',
            executable='rplidar_composition',
            name='rplidar_front',
            namespace='front_lidar',
            parameters=[{
                'serial_port': '/dev/ttyUSB0',
                'frame_id': 'front_laser_frame'
            }]
        ),
        
        # Secondary LiDAR (rear) - if available
        Node(
            package='rplidar_ros', 
            executable='rplidar_composition',
            name='rplidar_rear',
            namespace='rear_lidar',
            parameters=[{
                'serial_port': '/dev/ttyUSB1',
                'frame_id': 'rear_laser_frame'
            }]
        ),
        
        # Laser scan merger
        Node(
            package='ira_laser_tools',
            executable='laserscan_multi_merger',
            name='laser_merger',
            parameters=[{
                'destination_frame': 'base_link',
                'cloud_destination_topic': '/merged_scan',
                'scan_destination_topic': '/scan_merged',
                'laserscan_topics': ['/front_lidar/scan', '/rear_lidar/scan']
            }]
        )
    ])
```

### Performance Monitoring
```python
# lidar_monitor.py - Performance monitoring node
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import time

class LidarMonitor(Node):
    def __init__(self):
        super().__init__('lidar_monitor')
        self.subscription = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
        self.last_scan_time = time.time()
        
        # Performance metrics
        self.scan_count = 0
        self.average_frequency = 0.0
        self.range_errors = 0
        
    def scan_callback(self, msg):
        current_time = time.time()
        dt = current_time - self.last_scan_time
        
        # Calculate frequency
        if dt > 0:
            frequency = 1.0 / dt
            self.average_frequency = (
                self.average_frequency * self.scan_count + frequency
            ) / (self.scan_count + 1)
        
        # Check for range errors
        valid_ranges = [r for r in msg.ranges 
                       if msg.range_min <= r <= msg.range_max]
        if len(valid_ranges) < len(msg.ranges) * 0.8:
            self.range_errors += 1
            
        self.scan_count += 1
        self.last_scan_time = current_time
        
        # Log statistics every 100 scans
        if self.scan_count % 100 == 0:
            self.get_logger().info(
                f'LiDAR Stats: {self.average_frequency:.1f}Hz, '
                f'{self.range_errors} range errors'
            )
```

## Troubleshooting

### Hardware Issues

1. **LiDAR Not Detected**
   ```bash
   # Check USB connection
   lsusb | grep CP2102
   
   # Verify device permissions
   ls -la /dev/ttyUSB0
   sudo chmod 666 /dev/ttyUSB0
   
   # Test serial communication
   sudo apt install minicom
   minicom -D /dev/ttyUSB0 -b 256000
   ```

2. **Rotation Issues**
   - Check for physical obstructions
   - Verify power supply stability
   - Clean optical components

3. **Poor Scan Quality**
   - Ensure adequate lighting (>50 Lux)
   - Remove reflective surfaces from environment
   - Check for electromagnetic interference

### Software Issues

1. **Driver Not Starting**
   ```bash
   # Check ROS2 environment
   source /opt/ros/humble/setup.bash
   
   # Verify package installation
   ros2 pkg list | grep rplidar
   
   # Check node status
   ros2 node list
   ros2 node info /rplidar_node
   ```

2. **No Scan Data**
   ```bash
   # Check topic publication
   ros2 topic list | grep scan
   ros2 topic hz /scan
   
   # Monitor driver logs
   ros2 launch rplidar_ros rplidar_launch.py --ros-args --log-level DEBUG
   ```

3. **TF Frame Issues**
   ```bash
   # Check TF tree
   ros2 run tf2_tools view_frames.py
   
   # Verify frame transforms
   ros2 run tf2_ros tf2_echo base_link laser_frame
   ```

## Performance Optimization

### Hardware Optimization
- **USB Port Selection**: Use USB 3.0 port for stable connection
- **Cable Management**: Use short, high-quality USB cables
- **EMI Shielding**: Minimize electromagnetic interference sources

### Software Optimization
```yaml
# Optimized parameters for performance
rplidar_node:
  ros__parameters:
    # Reduce computational load
    frequency: 8.0  # Slightly lower for CPU savings
    angle_compensate: false  # Disable if not needed
    
    # Filter settings
    range_filter_enable: true
    range_filter_min: 0.15  # Filter very close readings
    range_filter_max: 10.0  # Limit max range for processing
    
    # Quality settings
    scan_quality_threshold: 15  # Minimum quality threshold
```

### System Integration
- **CPU Affinity**: Pin driver to specific CPU core
- **Priority**: Set real-time priority for driver process
- **Memory**: Use memory locking to prevent swapping

## Safety Considerations

### Laser Safety
- **Class 1 Laser**: Eye-safe under normal operating conditions
- **Maintenance**: Regular cleaning of optical components
- **Inspection**: Periodic check for physical damage

### Operational Safety
- **Emergency Stop**: LiDAR data integrated with EPO system
- **Obstacle Detection**: Real-time collision avoidance
- **Range Validation**: Filter invalid measurements

## Integration Examples

### Basic SLAM Example
```bash
# Terminal 1: Launch LiDAR
ros2 launch rplidar_ros rplidar_launch.py

# Terminal 2: Launch SLAM
ros2 launch slam_toolbox online_async_launch.py

# Terminal 3: Visualize
rviz2 -d ~/ros2_ws/src/sherpa_config/rviz/slam_view.rviz

# Terminal 4: Manual control for exploration
ros2 launch edubot_bringup teleop_launch.py
```

## References and Documentation

### Official Documentation
- [RPLidar SDK](https://github.com/Slamtec/rplidar_sdk)
- [RPLidar ROS2 Driver](https://github.com/Slamtec/rplidar_ros)
- [ROS2 sensor_msgs](https://docs.ros.org/en/humble/p/sensor_msgs/)

### Integration Guides
- [Navigation Integration](../2D_Nav/README.md)
- [SLAM Configuration](../v-slam/README.md)
- [Multi-Sensor Fusion](../computer-vision/README.md)
