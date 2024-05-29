## Follow the instructions given below sequentially to setup VLP32-C LiDAR from Velodyne on your Ubuntu Laptop

install ros2 humble [^2]

git clone https://github.com/ros-drivers/velodyne.git

set static IP: 192.168.1.100, subnet mask: 255.255.255.0, gateway: 192.168.1.1

git checkout humble-devel

source /opt/ros/humble/setup.bash

sudo apt install ros-humble-diagnostic-updater

sudo apt install libpcap-dev

colcon build --symlink-install

ros2 launch velodyne velodyne-all-nodes-VLP32C-launch.py

Open 2nd terminal

ros2 run tf2_ros static_transform_publisher "0" "0" "0" "0" "0" "0" "map" "velodyne"

Open 3rd terminal

source /opt/ros/humble/setup.bash

rviz2

click on add at the bottom part of the screen

select add by topic and select pointcloud2
