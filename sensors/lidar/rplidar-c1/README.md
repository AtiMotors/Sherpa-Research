Reference Repository : https://github.com/Slamtec/sllidar_ros2

# How to Create a ROS2 workspace
### example, choose the directory name ros2_ws, for "development workspace" :

mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

## Compile & Install sllidar_ros2 package
git clone https://github.com/Slamtec/sllidar_ros2.git


## Build sllidar_ros2 package
cd ~/ros2_ws/
source /opt/ros/<rosdistro>/setup.bash
colcon build --symlink-install

## Package environment setup

source ./install/setup.bash

### Note: Add permanent workspace environment variables. It's convenientif the ROS2 environment variables are automatically added to your bash session every time a new shell is launched:

## Create udev rules for rplidar
sllidar_ros2 running requires the read and write permissions of the serial device. You can manually modify it with the following command:

sudo chmod 777 /dev/ttyUSB0

### But a better way is to create a udev rule:

cd src/rpldiar_ros/
source scripts/create_udev_rules.sh

## The command for RPLIDAR C1 is :
ros2 launch sllidar_ros2 view_sllidar_c1_launch.py
