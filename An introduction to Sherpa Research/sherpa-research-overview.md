# An Introduction to Sherpa Research

## Overview
The Sherpa RP (Research Platform) is a modular, mobile robotic platform designed for educational, research, and light-duty material transport applications. This repository provides comprehensive documentation, code examples, and resources for developing with the Sherpa RP platform.

## Platform Specifications

### Physical Characteristics
- **Net Weight**: 51 kg (44 kg robot + 7 kg battery)
- **Payload Capacity**: 
  - Top Load: 51 kg
  - Towing Capacity: 100 kg
- **Travel Speed**: 0.7 m/s
- **Chassis**: Aluminum 2020 channels with sheet metal shell
- **Wheels**: Two 20cm cast iron wheels with solid rubber tires

### Operational Environment
- **Temperature Range**: +5°C to +40°C
- **Minimum Illumination**: 50 Lux
- **Maximum Slope**: 4% with load, 10% without load
- **Continuous Operation**: Up to 12 hours
- **Brief Outdoor Use**: Down to -10°C (max 30 minutes)

### Safety Features
- Dual Schneider Electric Emergency Power Off (EPO) buttons
- 48V lithium-ion NMC battery with BMS protection
- Electromagnetic brake system integrated with motors
- Power management with contactors and safety monitoring

## System Architecture

### Hardware Components
1. **Single Board Computer (SBC)**:
   - Raspberry Pi 5 (8GB/16GB) - Standard
   - NVIDIA Jetson Orin Nano - Optional

2. **Sensors & Peripherals**:
   - SlamTec RPLidar C1 - 2D LiDAR mapping
   - Intel RealSense D435i - 3D depth perception and IMU
   - Sony DualShock 4 - Manual control
   - Waveshare USB Hub - Additional peripheral connectivity

3. **Motor System**:
   - Dual BLDC motors with integrated Twara drivers
   - CAN 2.0b communication protocol
   - 24V/48V operation modes
   - 750W rated power per motor

### Software Stack
- **Operating System**: Ubuntu 22.04 LTS (in Docker container)
- **ROS Distribution**: ROS2 Humble Hawksbill
- **Navigation**: Nav2 stack with Cartographer SLAM
- **Containerization**: Docker-based deployment
- **Communication**: CAN bus for motor control

## Safety Guidelines

### CAUTION
- Maximum payload limits must never be exceeded
- Payload must be securely fastened to designated mounting points
- Robot must be fully initialized before operation
- EPO system must remain accessible at all times

### Prohibited Operations
- Carrying or lifting passengers
- Operating in explosive atmospheres
- Operation during adverse weather (outdoor use)
- Exceeding specified environmental limits

### Permitted Operations
- Autonomous and manual navigation in designated areas
- Transporting educational and research materials
- Indoor and controlled outdoor environments
- Research and development activities

## Getting Started

### Prerequisites
- Ubuntu 22.04 LTS
- ROS2 Humble
- Docker Engine
- Basic knowledge of ROS2 and robotics

### Quick Start
1. Clone this repository
2. Follow the setup instructions in the appropriate SBC directory
3. Build and deploy the Docker containers
4. Initialize the robot and run basic tests

### Support
- **Email**: support@atimotors.com
- **Documentation**: This repository
- **Community**: GitHub Issues and Discussions

## Repository Structure
This repository is organized into the following main sections:
- **Navigation**: 2D/3D navigation and SLAM implementations
- **Control**: Manual and autonomous control systems  
- **Hardware**: CAD files, schematics, and hardware documentation
- **Sensors**: Integration guides for various sensor types
- **Simulation**: Virtual environments for testing and development
- **Companion Computers**: Setup guides for different SBC options

## Contributing
We welcome contributions to the Sherpa Research platform. Please review our contribution guidelines and open source community standards.

## License
Copyright © Ati Motors Inc. - See individual files for specific licensing terms.