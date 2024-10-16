
My IMU Driver is publishing the following data : 

Orientations (Quarternion)
Linear Acceleration (x,y,z)
Angular Velocities (x,y,z)
Magnetic Field (x,y,z)

The following command has to be run to get the IMU Driver working on the emulator as follows : 

ros2 launch imu_driver imu_launch.py port:=/dev/pts/<emulator_number>

To run it on hardware : ros2 launch imu_driver imu_launch.py port:=/dev/<ttyUSB/ACM/any other port assigned> baudrate:="115200" frequency:=40

![image info](LAB3/src/IMU_Driver.png)

