# NVIDIA Jetson Orin Nano - Sherpa RP

## Overview
This directory contains setup instructions, configurations, and documentation for using the NVIDIA Jetson Orin Nano as the Single Board Computer (SBC) on the Sherpa RP platform. The Jetson Orin Nano provides enhanced AI computing capabilities for advanced computer vision, machine learning, and autonomous navigation tasks.

## Hardware Specifications

### NVIDIA Jetson Orin Nano Developer Kit
- **CPU**: 6-core ARM Cortex-A78AE @ 1.5GHz
- **GPU**: 1024-core NVIDIA Ampere GPU with 32 Tensor cores
- **AI Performance**: 40 TOPS (Tera Operations Per Second)
- **Memory**: 8GB 128-bit LPDDR5 @ 102.4 GB/s
- **Storage**: microSD slot, M.2 Key M NVMe SSD slot
- **Connectivity**:
  - 4x USB 3.2 Type-A
  - 1x USB-C (data only)
  - Gigabit Ethernet
  - WiFi 802.11ac + Bluetooth 5.2
  - 40-pin GPIO header
  - 2x MIPI CSI camera connectors
  - DisplayPort 1.4a output

### Power Requirements
- **Input Voltage**: 5V DC (USB-C) / 7-20V (barrel jack)
- **Power Consumption**: 7-15W typical operation
- **Power Modes**: Multiple performance modes (5W, 10W, 15W, 20W)
- **Sherpa RP Integration**: Powered by 24V-16V converter

## Software Stack

### Operating System
- **JetPack**: 5.1.4 (recommended for Sherpa RP)
- **Ubuntu**: 20.04 LTS (JetPack base)
- **Kernel**: Linux 5.10
- **CUDA**: 11.4
- **TensorRT**: 8.4
- **OpenCV**: 4.5.0 with CUDA support

### Container Environment
- **ROS2**: Humble Hawksbill (in Docker container)
- **Container OS**: Ubuntu 22.04 LTS
- **Runtime**: NVIDIA Container Runtime
- **GPU Access**: CUDA/TensorRT acceleration available

## Installation and Setup

### Prerequisites
- **Host PC**: Ubuntu 20.04 native installation (VM not supported)
- **NVIDIA SDK Manager**: Latest version
- **Internet Connection**: Required for JetPack download
- **MicroSD Card**: 64GB+ Class 10 or better
- **USB-C Cable**: For flashing and initial setup

### Step 1: Flash JetPack OS
```bash
# On host PC - Install SDK Manager
wget https://developer.nvidia.com/nvidia-sdk-manager
sudo dpkg -i sdkmanager_[version]_amd64.deb
sudo apt-get install -f

# Launch SDK Manager
sdkmanager

# Follow GUI wizard:
# 1. Select Jetson Orin Nano Developer Kit
# 2. Choose JetPack 5.1.4
# 3. Flash to microSD card or eMMC
# 4. Complete initial setup
```

### Step 2: Initial System Configuration
```bash
# After first boot on Jetson
sudo apt update && sudo apt upgrade -y

# Install development tools
sudo apt install build-essential cmake git

# Install CAN utilities (required for motor control)
sudo apt install can-utils

# Install additional dependencies
sudo apt install openssl libssl-dev bc
```

### Step 3: Docker Installation
```bash
# Install Docker with NVIDIA runtime support
distribution=$(. /etc/os-release;echo $ID$VERSION_ID) \
   && curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | sudo apt-key add - \
   && curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | sudo tee /etc/apt/sources.list.d/nvidia-docker.list

sudo apt update
sudo apt install docker.io nvidia-docker2
sudo systemctl restart docker

# Add user to docker group
sudo usermod -aG docker $USER

# Reboot to apply changes
sudo reboot
```

### Step 4: Sherpa RP Software Setup
```bash
# Clone Sherpa RP repository
cd ~/
git clone https://github.com/AtiMotors/Sherpa-Research.git ati.Sherpa_RP

# Navigate to Jetson Docker directory
cd ~/ati.Sherpa_RP/docker/ros2_humble_orin_nano/

# Build Docker image with NVIDIA runtime
docker build -t ros2_humble_orin_nano .

# Configure volume mapping in compose.yaml
# Edit the volumes section to point to your ros2_ws directory
volumes:
  - ~/ati.Sherpa_RP/ros2_ws:/home/ros/ros2_ws

# Launch container with GPU support
docker compose up -d

# Enter container
docker exec -it SherpaRP /bin/bash
```

### Step 5: ROS2 Environment Setup
```bash
# Inside Docker container
cd ~/ros2_ws

# Update package lists
sudo apt update && sudo apt upgrade

# Install ROS2 console bridge
sudo apt install ros-humble-console-bridge-vendor

# Initialize rosdep
sudo rosdep init && rosdep update

# Install workspace dependencies
rosdep install --from-paths src --ignore-src -r -y

# Build workspace
colcon build

# Source environment
source install/setup.bash
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
```

## Performance Configuration

### Power Mode Selection
```bash
# View available power modes
sudo nvpmodel -q

# Set to maximum performance (20W mode)
sudo nvpmodel -m 0

# Set to balanced mode (15W mode) - recommended for Sherpa RP
sudo nvpmodel -m 1

# Set to power-saving mode (5W mode)
sudo nvpmodel -m 2

# Enable maximum CPU clock speeds
sudo jetson_clocks
```

### GPU Memory Allocation
```bash
# Monitor GPU memory usage
tegrastats

# Configure memory split for AI workloads
sudo systemctl disable nvargus-daemon  # If not using multiple cameras
sudo systemctl disable nvfancontrol     # If using custom cooling
```

### System Monitoring
```bash
# Install jtop for system monitoring
sudo -H pip3 install -U jetson-stats

# Launch monitoring interface
jtop

# Monitor temperatures and power consumption
watch -n 1 cat /sys/class/thermal/thermal_zone*/temp
```

## Network Configuration

### WiFi Setup
```bash
# Connect to WiFi network
sudo nmtui

# Or via command line
sudo nmcli device wifi connect "SSID" password "PASSWORD"

# Configure static IP (optional)
sudo nmcli connection modify "Wired connection 1" \
  ipv4.addresses 192.168.1.100/24 \
  ipv4.gateway 192.168.1.1 \
  ipv4.dns "8.8.8.8 8.8.4.4" \
  ipv4.method manual
```

### SSH Configuration
```bash
# Enable SSH server
sudo systemctl enable ssh
sudo systemctl start ssh

# Configure SSH key authentication
ssh-keygen -t rsa -b 4096
# Copy public key to development machines

# Access Jetson remotely
ssh ati@edubot-jetson.local
# or
ssh ati@<jetson-ip-address>
```

## AI and Computer Vision Integration

### CUDA Acceleration
```python
# Verify CUDA installation
import torch
print(f"CUDA Available: {torch.cuda.is_available()}")
print(f"CUDA Version: {torch.version.cuda}")
print(f"Device Count: {torch.cuda.device_count()}")

# Example: GPU-accelerated OpenCV
import cv2
print(f"OpenCV CUDA Support: {cv2.cuda.getCudaEnabledDeviceCount() > 0}")
```

### TensorRT Optimization
```python
# Example: Convert PyTorch model to TensorRT
import torch
import tensorrt as trt
from torch2trt import torch2trt

# Load model
model = torch.load('model.pth').cuda().eval()

# Convert to TensorRT
model_trt = torch2trt(model, [input_tensor], fp16_mode=True)

# Save optimized model
torch.save(model_trt.state_dict(), 'model_trt.pth')
```

### Computer Vision Pipeline
```python
# ROS2 node example with GPU acceleration
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class VisionProcessor(Node):
    def __init__(self):
        super().__init__('vision_processor')
        self.bridge = CvBridge()
        
        # Subscribe to camera feed
        self.subscription = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)
        
        # Initialize GPU matrices
        self.gpu_frame = cv2.cuda_GpuMat()
        self.gpu_processed = cv2.cuda_GpuMat()
        
    def image_callback(self, msg):
        # Convert ROS image to OpenCV
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        
        # Upload to GPU
        self.gpu_frame.upload(cv_image)
        
        # GPU-accelerated processing
        cv2.cuda.bilateralFilter(
            self.gpu_frame, self.gpu_processed, -1, 50, 50)
        
        # Download processed image
        processed_image = self.gpu_processed.download()
        
        # Further processing...
```

## Hardware Interfaces

### GPIO Control
```python
# GPIO control using Jetson.GPIO library
import Jetson.GPIO as GPIO
import time

# Set up GPIO pins
LED_PIN = 18
BUTTON_PIN = 16

GPIO.setmode(GPIO.BOARD)
GPIO.setup(LED_PIN, GPIO.OUT)
GPIO.setup(BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# Blink LED
for i in range(10):
    GPIO.output(LED_PIN, GPIO.HIGH)
    time.sleep(0.5)
    GPIO.output(LED_PIN, GPIO.LOW)
    time.sleep(0.5)

GPIO.cleanup()
```

### Camera Interface
```python
# CSI camera example with GStreamer
import cv2

# GStreamer pipeline for CSI camera
gst_pipeline = (
    "nvarguscamerasrc sensor-id=0 ! "
    "video/x-raw(memory:NVMM), width=1920, height=1080, "
    "format=NV12, framerate=30/1 ! "
    "nvvidconv ! video/x-raw, format=BGRx ! "
    "videoconvert ! video/x-raw, format=BGR ! "
    "appsink drop=1"
)

# Open camera
cap = cv2.VideoCapture(gst_pipeline, cv2.CAP_GSTREAMER)

while True:
    ret, frame = cap.read()
    if ret:
        cv2.imshow('Camera', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

cap.release()
cv2.destroyAllWindows()
```

### CAN Bus Integration
```bash
# Setup CAN interface
sudo modprobe can
sudo modprobe can_raw
sudo modprobe mttcan

# Configure CAN0 (requires hardware CAN transceiver)
sudo ip link set can0 type can bitrate 1000000
sudo ip link set up can0

# Test CAN communication
cansend can0 123#DEADBEEF
candump can0
```

## Thermal Management

### Cooling Solutions
- **Passive Cooling**: Heat sink (included with developer kit)
- **Active Cooling**: Optional fan (recommended for continuous operation)
- **Thermal Throttling**: Automatic performance reduction at high temperatures

### Temperature Monitoring
```bash
# Monitor temperatures
watch -n 1 "echo 'CPU:'; cat /sys/class/thermal/thermal_zone0/temp; \
            echo 'GPU:'; cat /sys/class/thermal/thermal_zone1/temp"

# Set temperature limits
sudo sh -c 'echo 85000 > /sys/class/thermal/thermal_zone0/trip_point_1_temp'
```

### Cooling Configuration
```python
# Custom fan control script
#!/usr/bin/env python3
import time
import Jetson.GPIO as GPIO

FAN_PIN = 32  # GPIO pin for fan control
TEMP_THRESHOLD = 55  # Temperature threshold in Celsius

GPIO.setmode(GPIO.BOARD)
GPIO.setup(FAN_PIN, GPIO.OUT)
pwm = GPIO.PWM(FAN_PIN, 1000)  # 1kHz PWM
pwm.start(0)

try:
    while True:
        with open('/sys/class/thermal/thermal_zone0/temp', 'r') as f:
            temp = int(f.read()) / 1000.0
        
        if temp > TEMP_THRESHOLD:
            fan_speed = min(100, (temp - TEMP_THRESHOLD) * 10)
            pwm.ChangeDutyCycle(fan_speed)
        else:
            pwm.ChangeDutyCycle(0)
        
        time.sleep(5)
except KeyboardInterrupt:
    pwm.stop()
    GPIO.cleanup()
```

## Troubleshooting

### Common Issues

1. **Boot Issues**
```bash
# Check boot logs
dmesg | grep -i error

# Verify power supply (minimum 3A at 5V)
# Check microSD card integrity
sudo fsck /dev/mmcblk0p1

# Recovery mode entry
# Hold recovery button while powering on
```

2. **Docker Issues**
```bash
# Verify NVIDIA runtime
docker run --rm --gpus all nvidia/cuda:11.4-base-ubuntu20.04 nvidia-smi

# Permission issues
sudo usermod -aG docker $USER
newgrp docker

# Container networking
docker network ls
docker network inspect bridge
```

3. **Performance Issues**
```bash
# Check power mode
sudo nvpmodel -q

# Monitor system resources
tegrastats
jtop

# Verify GPU usage
nvidia-smi
watch -n 1 nvidia-smi
```

4. **Camera Issues**
```bash
# List available cameras
ls /dev/video*
v4l2-ctl --list-devices

# Test CSI camera
gst-launch-1.0 nvarguscamerasrc sensor-id=0 ! \
  nvoverlaysink overlay-x=0 overlay-y=0 overlay-w=1920 overlay-h=1080

# USB camera test
gst-launch-1.0 v4l2src device=/dev/video0 ! \
  video/x-raw,width=640,height=480 ! xvimagesink
```

### Performance Optimization

1. **Disable Unnecessary Services**
```bash
# Disable GUI if running headless
sudo systemctl set-default multi-user.target

# Disable unnecessary services
sudo systemctl disable cups-browsed
sudo systemctl disable avahi-daemon
sudo systemctl disable bluetooth

# Optimize memory usage
sudo sh -c 'echo 1 > /proc/sys/vm/swappiness'
```

2. **Storage Optimization**
```bash
# Use NVMe SSD for better I/O performance
sudo gdisk /dev/nvme0n1
# Create ext4 partition and mount as /home/ati/ros2_ws

# Enable ZRAM
sudo apt install zram-config
```

## Integration with Sherpa RP Systems

### Motor Control Integration
```python
# CAN bus motor controller node
import can
from geometry_msgs.msg import Twist

class JetsonMotorController(Node):
    def __init__(self):
        super().__init__('motor_controller')
        self.bus = can.interface.Bus(channel='can0', bustype='socketcan')
        self.subscription = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)
    
    def cmd_vel_callback(self, msg):
        # Convert twist to differential drive
        left_speed = msg.linear.x - msg.angular.z * 0.3
        right_speed = msg.linear.x + msg.angular.z * 0.3
        
        # Send CAN commands to Twara motor drivers
        self.send_motor_command(0x141, left_speed)
        self.send_motor_command(0x142, right_speed)
```

### Sensor Fusion
```python
# Multi-sensor data fusion
class SensorFusion(Node):
    def __init__(self):
        super().__init__('sensor_fusion')
        
        # Subscribe to multiple sensors
        self.lidar_sub = self.create_subscription(
            LaserScan, '/scan', self.lidar_callback, 10)
        self.camera_sub = self.create_subscription(
            Image, '/camera/image_raw', self.camera_callback, 10)
        self.imu_sub = self.create_subscription(
            Imu, '/imu', self.imu_callback, 10)
        
        # Publish fused data
        self.obstacle_pub = self.create_publisher(
            PointCloud2, '/obstacles', 10)
```

## Development Workflow

### Remote Development
```bash
# VS Code remote development
# Install "Remote - SSH" extension
# Connect to jetson via SSH
code --folder-uri vscode-remote://ssh-remote+ati@edubot-jetson.local/home/ati/ros2_ws
```

### Debugging and Profiling
```bash
# CPU profiling
sudo perf record -g ros2 launch edubot_bringup robot_bringup.launch.py
sudo perf report

# GPU profiling
nvprof python3 vision_node.py

# Memory profiling
valgrind --tool=massif python3 navigation_node.py
```

## References and Documentation

### NVIDIA Resources
- [Jetson Orin Nano Developer Kit User Guide](https://developer.nvidia.com/embedded/jetson-orin-nano-developer-kit)
- [JetPack SDK Documentation](https://developer.nvidia.com/embedded/jetpack)
- [CUDA for Tegra](https://developer.nvidia.com/embedded/jetpack)

### Sherpa RP Integration
- [Raspberry Pi Alternative](../raspberry-pi/README.md)
- [ROS2 Configuration](../../ROS2/README.md)
- [Motor Control](../../controls/README.md)
- [Computer Vision](../../computer-vision/README.md)
