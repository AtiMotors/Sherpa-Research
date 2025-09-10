# Teleoperation (Manual Control) - Sherpa RP

## Overview
This directory contains packages and documentation for manual control of the Sherpa RP platform using a Sony DualShock 4 wireless controller. The teleoperation system allows operators to manually navigate the robot for testing, emergency override, and teaching purposes.

## Hardware Requirements

### Controller Specifications
- **Model**: Sony DualShock 4 Wireless Controller
- **Connection**: Bluetooth wireless
- **Range**: Up to 10 meters (line of sight)
- **Battery Life**: ~8 hours continuous use
- **Compatibility**: Linux, Windows, macOS

### Control Mapping
| Button/Stick | Function | Description |
|--------------|----------|-------------|
| Left Stick (C) | Forward/Backward | Vertical movement controls linear velocity |
| Right Stick (D) | Turn | Horizontal movement controls angular velocity |
| Share Button (A) | Status | Display current system status |
| Options Button (B) | Manual Mode Toggle | Enable/disable manual control |
| Home Button (E) | System Home | Return to idle state |
| A + E Together | Pairing Mode | Put controller in discoverable mode |

### Status Indicators
| LED State | Description |
|-----------|-------------|
| Flashing | Controller in pairing mode, discoverable by devices |
| Steady | Controller connected and operational |
| Off | Controller disconnected or in sleep mode |

## Software Architecture

### ROS2 Node Structure
```
┌─────────────────┐    ┌──────────────────┐    ┌─────────────────┐
│   joy_node      │───▶│  teleop_twist    │───▶│ robot_control   │
│ (Controller)    │    │     (Twist)      │    │   (CAN Bus)     │
└─────────────────┘    └──────────────────┘    └─────────────────┘
```

### Key ROS2 Topics
- **Input**: `/joy` (sensor_msgs/Joy)
- **Output**: `/cmd_vel` (geometry_msgs/Twist)
- **Status**: `/teleop_status` (std_msgs/Bool)

## Setup and Installation

### Step 1: Install Dependencies
```bash
# Install joystick drivers and ROS2 packages
sudo apt update
sudo apt install ros-humble-joy
sudo apt install ros-humble-teleop-twist-joy
sudo apt install ds4drv
```

### Step 2: Controller Pairing
```bash
# Put controller in pairing mode (Press A + E together)
# The LED should start flashing

# Enable Bluetooth on your system
sudo systemctl start bluetooth
sudo systemctl enable bluetooth

# Use bluetoothctl to pair
bluetoothctl
> scan on
> pair [CONTROLLER_MAC_ADDRESS]
> connect [CONTROLLER_MAC_ADDRESS]
> exit
```

### Step 3: Test Controller Connection
```bash
# Check if controller is detected
ls /dev/input/js*

# Test joystick input
jstest /dev/input/js0

# Or use ROS2 joystick node
ros2 run joy joy_node
```

## Launch Configuration

### Basic Teleop Launch File
Create `launch/teleop_launch.launch.py`:

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'joy_dev',
            default_value='/dev/input/js0',
            description='Joystick device path'
        ),
        
        # Joy node for controller input
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
            parameters=[
                {'dev': LaunchConfiguration('joy_dev')},
                {'deadzone': 0.1},
                {'autorepeat_rate': 20.0}
            ]
        ),
        
        # Teleop twist joy for velocity conversion
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist_joy',
            output='screen',
            parameters=[
                {'axis_linear.x': 1},  # Left stick vertical
                {'axis_angular.yaw': 3},  # Right stick horizontal
                {'scale_linear.x': 0.7},  # Max linear speed (m/s)
                {'scale_angular.yaw': 2.0},  # Max angular speed (rad/s)
                {'enable_button': 1},  # Options button (B)
                {'enable_turbo_button': -1}  # Disabled
            ]
        )
    ])
```

### Advanced Teleop with Safety Features
```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Joy node with enhanced parameters
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
            parameters=[
                {'dev': '/dev/input/js0'},
                {'deadzone': 0.15},
                {'autorepeat_rate': 20.0},
                {'coalesce_interval': 0.01}
            ]
        ),
        
        # Custom teleop node with safety features
        Node(
            package='sherpa_teleop',
            executable='safe_teleop_node',
            name='safe_teleop',
            output='screen',
            parameters=[
                # Movement parameters
                {'max_linear_velocity': 0.7},
                {'max_angular_velocity': 2.0},
                {'acceleration_limit': 1.0},
                {'deceleration_limit': 2.0},
                
                # Safety parameters  
                {'emergency_stop_button': 4},  # L1 button
                {'deadman_button': 1},  # Options button
                {'timeout_duration': 0.5},  # Command timeout (s)
                
                # Control sensitivity
                {'linear_scale_turbo': 1.0},
                {'linear_scale_normal': 0.5},
                {'angular_scale_turbo': 1.5},
                {'angular_scale_normal': 1.0}
            ]
        )
    ])
```

## Manual Control Procedures

### Startup Sequence
1. **Power On Sherpa RP**: Press power button and wait for initialization
2. **Connect Controller**: Ensure controller is paired and connected
3. **Launch Teleop**: Start the teleoperation launch file
4. **Enable Manual Mode**: Press Options button (B) to activate control

```bash
# Terminal 1: Launch main robot systems
ros2 launch edubot_bringup robot_bringup.launch.py

# Terminal 2: Start joystick node
ros2 run joy joy_node

# Terminal 3: Enable teleop control
ros2 launch edubot_bringup teleop_launch.launch.py
```

### Control Operation
1. **Hold Enable Button**: Keep Options button (B) pressed for control
2. **Forward/Backward**: Use left stick vertical axis
3. **Turning**: Use right stick horizontal axis
4. **Emergency Stop**: Release enable button or press EPO hardware button

### Status Monitoring
```bash
# Check teleop status
ros2 topic echo /teleop_status

# Monitor controller input
ros2 topic echo /joy

# View velocity commands
ros2 topic echo /cmd_vel
```

## Safety Features

### Software Safety
- **Deadman Switch**: Requires constant button press (Options/B) to maintain control
- **Command Timeout**: Automatically stops robot if no commands received within 0.5 seconds
- **Velocity Limiting**: Enforces maximum safe speeds
- **Smooth Acceleration**: Prevents sudden velocity changes

### Hardware Safety
- **EPO Integration**: Hardware emergency stop overrides all software control
- **Battery Monitoring**: Warns of low battery conditions
- **Motor Safety**: Electromagnetic brakes engage during emergency stops

### Safety Procedures
```bash
# EMERGENCY STOP SEQUENCE
# 1. Release all controller buttons immediately
# 2. Press hardware EPO button if needed
# 3. Check robot status before resuming

# NORMAL SHUTDOWN
# 1. Release enable button (Options/B)  
# 2. Wait for robot to stop completely
# 3. Power down using proper shutdown sequence
sudo shutdown 0
```

## Troubleshooting

### Controller Connection Issues

1. **Controller Not Detected**
```bash
# Check USB/Bluetooth connection
lsusb | grep Sony
bluetoothctl info [MAC_ADDRESS]

# Restart Bluetooth service
sudo systemctl restart bluetooth
```

2. **Joy Node Not Publishing**
```bash
# Check device permissions
ls -la /dev/input/js0
sudo chmod 666 /dev/input/js0

# Test with jstest utility
jstest /dev/input/js0
```

3. **No Response from Robot**
```bash
# Check if teleop node is running
ros2 node list | grep teleop

# Verify cmd_vel topic
ros2 topic hz /cmd_vel
ros2 topic echo /cmd_vel

# Check enable button status
ros2 topic echo /joy
```

### Common Solutions

1. **Controller Lag or Delay**
   - Reduce joystick deadzone parameter
   - Increase autorepeat rate
   - Check Bluetooth signal strength

2. **Jerky Movement**
   - Increase deadzone to filter noise
   - Implement velocity smoothing
   - Check controller calibration

3. **Intermittent Control Loss**
   - Replace controller batteries
   - Check for interference sources
   - Verify Bluetooth connection stability

## Advanced Configuration

### Custom Control Mapping
Create custom button mappings in `config/teleop_params.yaml`:

```yaml
teleop_twist_joy:
  ros__parameters:
    axis_linear:
      x: 1  # Left stick up/down
    axis_angular:
      yaw: 3  # Right stick left/right
    scale_linear:
      x: 0.7  # Max linear speed
    scale_angular:
      yaw: 2.0  # Max angular speed
    enable_button: 1  # Options button
    enable_turbo_button: 5  # R1 button for turbo mode
    
    # Button mapping for additional functions
    buttons:
      emergency_stop: 4  # L1 button
      horn: 0  # X button  
      lights: 2  # Square button
      mode_switch: 3  # Triangle button
```

### Integration with Autonomous Mode
```python
# Example: Seamless transition between manual and autonomous control
def manual_override_callback(self, msg):
    if self.is_manual_enabled(msg):
        # Disable autonomous navigation
        self.nav_stack.cancel_all_goals()
        self.switch_to_manual_mode()
    else:
        # Resume autonomous operation
        self.switch_to_autonomous_mode()
```

## Performance Metrics

### Control Responsiveness
- **Input Latency**: < 50ms from controller to motor command
- **Update Rate**: 20Hz controller sampling
- **Command Rate**: 10Hz velocity commands to motors

### Safety Response Times
- **Manual Stop**: < 100ms (software)
- **Emergency Stop**: < 10ms (hardware EPO)
- **Timeout Response**: 500ms maximum

## Integration Examples

See the following directories for integration examples:
- `../autonomy/` - Switching between manual and autonomous modes
- `../ROS2/` - ROS2-specific implementation details
- `../companion-computer/` - SBC-specific setup instructions

## References
- [ROS2 Joy Package Documentation](https://github.com/ros-drivers/joystick_drivers)
- [Sony DualShock 4 Linux Driver](https://github.com/chrippa/ds4drv)
- [Nav2 Manual Override Tutorial](https://navigation.ros.org/)
