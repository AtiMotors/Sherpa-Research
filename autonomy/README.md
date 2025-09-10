# Autonomous Navigation - Sherpa RP

## Overview
This directory contains packages, algorithms, and documentation for autonomous navigation capabilities of the Sherpa RP platform. The autonomy system enables the robot to navigate independently using sensor data, pre-built maps, and real-time path planning while ensuring safe operation.

## System Architecture

### Navigation Framework
The Sherpa RP uses the ROS2 Nav2 (Navigation2) stack as the foundation for autonomous navigation:

```
┌─────────────────┐    ┌──────────────────┐    ┌─────────────────┐
│    Sensors      │───▶│   Perception     │───▶│  Path Planning  │
│ (LiDAR, Camera) │    │   & Mapping      │    │   & Control     │
└─────────────────┘    └──────────────────┘    └─────────────────┘
         │                       │                       │
         │              ┌──────────────────┐             │
         └─────────────▶│   Localization   │◄────────────┘
                        │      (AMCL)      │
                        └──────────────────┘
                                 │
                        ┌──────────────────┐
                        │  Safety Monitor  │
                        │   & Emergency    │
                        │     Override     │
                        └──────────────────┘
```

### Core Components

#### 1. Navigation Server (Nav2)
- **Global Planner**: Calculates optimal path from start to goal
- **Local Planner**: Real-time obstacle avoidance and path following
- **Behavior Trees**: Decision-making framework for complex behaviors
- **Recovery Behaviors**: Handling of navigation failures

#### 2. Localization System
- **AMCL**: Adaptive Monte Carlo Localization for pose estimation
- **Odometry**: Wheel encoder and IMU sensor fusion
- **Map Matching**: Laser scan matching against known maps

#### 3. Perception Stack
- **Costmaps**: Dynamic obstacle representation
- **Sensor Processing**: LiDAR and camera data integration
- **Object Detection**: Identification of dynamic obstacles

#### 4. Safety Systems
- **Collision Avoidance**: Real-time obstacle detection and avoidance
- **EPO Integration**: Emergency stop functionality
- **Velocity Limits**: Speed restrictions for safe operation
- **Manual Override**: Seamless transition to manual control

## Installation and Setup

### Prerequisites
```bash
# Ensure ROS2 Humble and Nav2 are installed
sudo apt update
sudo apt install ros-humble-navigation2
sudo apt install ros-humble-nav2-bringup
sudo apt install ros-humble-slam-toolbox
sudo apt install ros-humble-robot-localization
```

### Workspace Setup
```bash
# Build navigation packages
cd ~/ros2_ws
colcon build --packages-select \
    edubot_navigation \
    edubot_localization \
    edubot_planning \
    safety_monitor

# Source environment
source install/setup.bash
```

## Configuration Files

### Navigation Parameters
Core navigation configuration in `config/nav2_params.yaml`:
```yaml
# Navigation Server Configuration
bt_navigator:
  ros__parameters:
    use_sim_time: False
    global_frame: map
    robot_base_frame: base_link
    odom_topic: /odom
    bt_loop_duration: 10
    default_server_timeout: 20
    
    # Behavior Tree Configuration
    plugin_lib_names:
      - nav2_compute_path_to_pose_action_bt_node
      - nav2_follow_path_action_bt_node
      - nav2_back_up_action_bt_node
      - nav2_spin_action_bt_node
      - nav2_wait_action_bt_node
      - nav2_clear_costmap_service_bt_node

# Controller Server
controller_server:
  ros__parameters:
    use_sim_time: False
    controller_frequency: 20.0
    
    # DWB Local Planner
    FollowPath:
      plugin: "dwb_core::DWBLocalPlanner"
      
      # Velocity limits
      max_vel_x: 0.7      # Maximum linear velocity (m/s)
      min_vel_x: -0.7     # Minimum linear velocity (m/s)
      max_vel_y: 0.0      # Holonomic robots only
      min_vel_y: 0.0
      max_vel_theta: 2.0  # Maximum angular velocity (rad/s)
      min_vel_theta: -2.0
      
      # Acceleration limits
      acc_lim_x: 1.0      # Linear acceleration limit (m/s²)
      acc_lim_y: 0.0
      acc_lim_theta: 2.0  # Angular acceleration limit (rad/s²)
      
      # Path following
      path_distance_bias: 64.0
      goal_distance_bias: 24.0
      occdist_scale: 0.01
      forward_point_distance: 0.325
      
      # Trajectory sampling
      sim_time: 2.0
      vx_samples: 12
      vy_samples: 1
      vtheta_samples: 20

# Global Planner
planner_server:
  ros__parameters:
    use_sim_time: False
    planner_plugins: ["GridBased"]
    
    GridBased:
      plugin: "nav2_navfn_planner/NavfnPlanner"
      tolerance: 0.5
      use_astar: false
      allow_unknown: true

# Recovery Behaviors
recoveries_server:
  ros__parameters:
    use_sim_time: False
    costmap_topic: local_costmap/costmap_raw
    footprint_topic: local_costmap/published_footprint
    cycle_frequency: 10.0
    
    recovery_plugins: ["spin", "backup", "wait"]
    
    spin:
      plugin: "nav2_recoveries/Spin"
    backup:
      plugin: "nav2_recoveries/BackUp"
    wait:
      plugin: "nav2_recoveries/Wait"
```

### Costmap Configuration
Local and global costmap settings for obstacle representation:
```yaml
# Global Costmap (for global planning)
global_costmap:
  global_costmap:
    ros__parameters:
      update_frequency: 1.0
      publish_frequency: 1.0
      global_frame: map
      robot_base_frame: base_link
      resolution: 0.05
      width: 100
      height: 100
      
      # Robot footprint
      footprint: "[[-0.35, -0.3], [-0.35, 0.3], [0.35, 0.3], [0.35, -0.3]]"
      
      plugins: ["static_layer", "obstacle_layer", "inflation_layer"]
      
      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_subscribe_transient_local: True
        
      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: True
        observation_sources: scan
        
        scan:
          topic: /scan
          max_obstacle_height: 2.0
          clearing: True
          marking: True
          data_type: "LaserScan"
          raytrace_max_range: 10.0
          raytrace_min_range: 0.0
          obstacle_max_range: 8.0
          obstacle_min_range: 0.0
          
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 10.0
        inflation_radius: 0.55

# Local Costmap (for local planning)
local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 5.0
      publish_frequency: 2.0
      global_frame: odom
      robot_base_frame: base_link
      rolling_window: true
      width: 6
      height: 6
      resolution: 0.05
      
      plugins: ["obstacle_layer", "inflation_layer"]
      # Same plugin configuration as global costmap
```

## Launch Configuration

### Complete Autonomous Navigation
Main navigation launch file `launch/autonomous_navigation.launch.py`:
```python
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Package directories
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    edubot_dir = get_package_share_directory('edubot_navigation')
    
    # Configuration files
    nav2_params = os.path.join(edubot_dir, 'config', 'nav2_params.yaml')
    map_file = os.path.join(edubot_dir, 'maps', 'warehouse_map.yaml')
    
    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'map', default_value=map_file,
            description='Full path to map yaml file'),
        DeclareLaunchArgument(
            'params_file', default_value=nav2_params,
            description='Full path to param file'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use simulation time'),
        
        # Robot state publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}]
        ),
        
        # Localization
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                get_package_share_directory('localization_server'),
                '/launch/localization.launch.py'
            ]),
            launch_arguments={
                'map': LaunchConfiguration('map'),
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }.items()
        ),
        
        # Navigation
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                nav2_bringup_dir, '/launch/navigation_launch.py'
            ]),
            launch_arguments={
                'params_file': LaunchConfiguration('params_file'),
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }.items()
        ),
        
        # Safety monitor
        Node(
            package='safety_monitor',
            executable='safety_node',
            name='safety_monitor',
            output='screen',
            parameters=[nav2_params]
        ),
        
        # Behavior coordinator
        Node(
            package='edubot_behavior',
            executable='behavior_coordinator',
            name='behavior_coordinator',
            output='screen'
        )
    ])
```

## Operation Procedures

### Pre-Navigation Checklist
1. **Map Verification**: Ensure current map matches environment
2. **Sensor Check**: Verify all sensors are operational
3. **Battery Level**: Confirm adequate battery charge
4. **Safety Systems**: Test EPO buttons and manual override
5. **Clear Path**: Ensure initial path to goal is obstacle-free

### Starting Autonomous Navigation
```bash
# Step 1: Launch robot systems
ros2 launch edubot_bringup robot_bringup.launch.py

# Step 2: Start localization and navigation
ros2 launch edubot_navigation autonomous_navigation.launch.py

# Step 3: Set initial pose in RViz
# Click "2D Pose Estimate" and click/drag on map

# Step 4: Send navigation goal
# Click "2D Nav Goal" in RViz and click target location
```

### Command Line Navigation
```bash
# Send goal via command line
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 5.0, y: 2.0, z: 0.0}, 
  orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}}"

# Cancel current goal
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  --feedback
```

### Monitoring Navigation
```bash
# Check navigation status
ros2 topic echo /navigate_to_pose/_action/status

# Monitor robot pose
ros2 topic echo /amcl_pose

# View costmap data
ros2 topic echo /global_costmap/costmap_updates
ros2 topic echo /local_costmap/costmap_updates

# Check for obstacles
ros2 topic echo /scan
```

## Advanced Features

### Waypoint Navigation
Sequential navigation through multiple points:
```python
import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped

class WaypointNavigator:
    def __init__(self):
        rclpy.init()
        self.navigator = BasicNavigator()
        
    def create_pose(self, x, y, yaw):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.navigator.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.orientation.z = sin(yaw / 2)
        pose.pose.orientation.w = cos(yaw / 2)
        return pose
        
    def navigate_waypoints(self, waypoints):
        # Wait for navigation to activate
        self.navigator.waitUntilNav2Active()
        
        # Follow waypoints
        self.navigator.followWaypoints(waypoints)
        
        # Monitor progress
        while not self.navigator.isTaskComplete():
            feedback = self.navigator.getFeedback()
            print(f'Distance to goal: {feedback.distance_remaining}')
        
        result = self.navigator.getResult()
        if result == NavigationResult.SUCCEEDED:
            print('Navigation completed successfully!')
        else:
            print(f'Navigation failed: {result}')

# Usage example
waypoints = [
    create_pose(2.0, 1.0, 0.0),    # First waypoint
    create_pose(4.0, 3.0, 1.57),  # Second waypoint  
    create_pose(0.0, 0.0, 3.14)   # Return to start
]

navigator = WaypointNavigator()
navigator.navigate_waypoints(waypoints)
```

### Dynamic Reconfiguration
Adjust navigation parameters during operation:
```bash
# Change maximum velocity
ros2 param set /controller_server max_vel_x 0.3

# Adjust global planner tolerance
ros2 param set /planner_server GridBased.tolerance 1.0

# Update costmap parameters
ros2 param set /global_costmap/global_costmap inflation_radius 0.8
```

### Behavior Trees
Custom decision-making logic using behavior trees:
```xml
<!-- behavior_tree.xml -->
<root main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <ReactiveSequence>
      <!-- Safety checks -->
      <Condition ID="IsBatteryOK"/>
      <Condition ID="IsPathClear"/>
      
      <!-- Navigation sequence -->
      <Sequence>
        <Action ID="ComputePathToPose"/>
        <Action ID="FollowPath"/>
      </Sequence>
      
      <!-- Recovery behaviors -->
      <ReactiveFallback>
        <Action ID="ClearEntireCostmap"/>
        <Action ID="Spin"/>
        <Action ID="BackUp"/>
        <Action ID="Wait"/>
      </ReactiveFallback>
    </ReactiveSequence>
  </BehaviorTree>
</root>
```

## Safety Systems

### Collision Avoidance
Multi-layered approach to prevent collisions:

1. **Sensor-Based Detection**:
   - LiDAR: 360° obstacle detection
   - Camera: Visual obstacle identification
   - Ultrasonic: Close-range detection

2. **Predictive Avoidance**:
   - Dynamic obstacle tracking
   - Trajectory prediction
   - Preemptive path planning

3. **Emergency Stop**:
   - Hardware EPO buttons
   - Software emergency brake
   - Immediate motor shutdown

### Safety Monitor Node
```python
class SafetyMonitor(Node):
    def __init__(self):
        super().__init__('safety_monitor')
        
        # Subscribers
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)
            
        # Emergency stop service
        self.emergency_stop_srv = self.create_service(
            Trigger, 'emergency_stop', self.emergency_stop_callback)
            
        # Safety parameters
        self.min_obstacle_distance = 0.3  # meters
        self.max_velocity = 0.7  # m/s
        self.emergency_active = False
        
    def scan_callback(self, msg):
        # Check for close obstacles
        min_range = min([r for r in msg.ranges 
                        if msg.range_min <= r <= msg.range_max])
        
        if min_range < self.min_obstacle_distance:
            self.trigger_emergency_stop()
    
    def cmd_vel_callback(self, msg):
        # Velocity limiting
        if abs(msg.linear.x) > self.max_velocity:
            self.get_logger().warn('Velocity limit exceeded')
            # Publish limited velocity
            
    def trigger_emergency_stop(self):
        if not self.emergency_active:
            self.emergency_active = True
            self.get_logger().error('EMERGENCY STOP TRIGGERED')
            # Stop robot and cancel navigation goals
```

## Performance Optimization

### Navigation Tuning
Key parameters for optimal performance:

1. **Controller Frequency**: 20Hz for responsive control
2. **Costmap Updates**: Balance frequency vs. computational load
3. **Planner Resolution**: Fine resolution for accuracy, coarse for speed
4. **Recovery Timeouts**: Quick recovery without excessive waiting

### Computational Optimization
```bash
# Set CPU governor for consistent performance  
sudo cpufreq-set -g performance

# Adjust process priorities
sudo nice -n -10 ros2 launch edubot_navigation autonomous_navigation.launch.py

# Monitor CPU and memory usage
htop
ros2 run resource_monitor resource_monitor_node
```

### Network Optimization
```bash
# Optimize ROS2 DDS settings
export RMW_IMPLEMENTATION=rmw_cyclonedx_cpp
export CYCLONEDX_URI='<CycloneDX><Domain><General><MaxMessageSize>65500</MaxMessageSize></General></Domain></CycloneDX>'
```

## Troubleshooting

### Common Navigation Issues

1. **Robot Won't Move**
   ```bash
   # Check navigation status
   ros2 node list | grep nav
   ros2 service call /bt_navigator/is_active std_srvs/srv/Empty
   
   # Verify goal reception
   ros2 topic echo /goal_pose
   ```

2. **Poor Localization**
   ```bash
   # Check AMCL pose confidence
   ros2 topic echo /amcl_pose
   
   # Adjust particle filter parameters
   ros2 param set /amcl max_particles 3000
   ```

3. **Path Planning Failures**
   ```bash
   # Check map and costmap
   ros2 topic echo /map
   ros2 service call /global_costmap/clear_entirely_global_costmap nav2_msgs/srv/ClearEntireCostmap
   ```

4. **Obstacle Avoidance Issues**
   ```bash
   # Monitor local costmap
   ros2 topic echo /local_costmap/costmap_updates
   
   # Check sensor data
   ros2 topic echo /scan
   ros2 topic hz /scan
   ```

### Debug Tools
```bash
# Navigation debugging
ros2 run nav2_util dump_params nav2_params_dump.yaml
ros2 launch nav2_bringup navigation_launch.py params_file:=debug_params.yaml

# RViz navigation debugging
rviz2 -d $(ros2 pkg prefix nav2_bringup)/share/nav2_bringup/rviz/nav2_default_view.rviz
```

## Integration Examples

### Mission Planning System
```python
class MissionPlanner(Node):
    def __init__(self):
        super().__init__('mission_planner')
        self.navigator = BasicNavigator()
        self.current_mission = None
        
    def execute_delivery_mission(self, pickup_point, delivery_point):
        """Execute a delivery mission"""
        mission_plan = [
            ('navigate', pickup_point, 'Moving to pickup location'),
            ('dock', None, 'Docking for pickup'),
            ('wait', 5.0, 'Loading cargo'),
            ('navigate', delivery_point, 'Moving to delivery location'),
            ('dock', None, 'Docking for delivery'),
            ('wait', 5.0, 'Unloading cargo'),
            ('navigate', 'home', 'Returning to base')
        ]
        
        self.execute_mission_plan(mission_plan)
    
    def execute_patrol_mission(self, patrol_points):
        """Execute a patrol mission"""
        while True:
            for point in patrol_points:
                if not self.navigate_to_point(point):
                    self.get_logger().error(f'Failed to reach patrol point {point}')
                    break
                time.sleep(30)  # Patrol dwell time
```

## Future Enhancements

### Planned Features
1. **Multi-Robot Coordination**: Fleet management capabilities
2. **Learning-Based Planning**: Adaptive path planning from experience
3. **Semantic Navigation**: Object and room-based navigation
4. **Voice Control**: Natural language navigation commands

### Research Areas
- **SLAM Improvements**: Real-time map updates during navigation
- **Human-Robot Interaction**: Safe navigation in human environments
- **Energy Optimization**: Battery-aware path planning
- **Robust Localization**: GPS-denied environment navigation

## References and Documentation

### Official Documentation
- [Nav2 Documentation](https://navigation.ros.org/)
- [ROS2 Navigation Tutorials](https://docs.ros.org/en/humble/Tutorials/Advanced/Navigation/Navigation-Main.html)
- [Behavior Trees for Robotics](https://navigation.ros.org/behavior_trees/index.html)

### Related Sherpa RP Documentation
- [2D Navigation](../2D_Nav/README.md)
- [SLAM Implementation](../v-slam/README.md)
- [Sensor Integration](../sensors/README.md)
- [Safety Systems](../controls/README.md)
