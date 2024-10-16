from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'port',
            default_value = '/dev/ttyUSB0',
            description = 'Serial'
        ),

        DeclareLaunchArgument(
            'baudrate',
            default_value = '115200',
            description = 'Baudrate'
        ),

        DeclareLaunchArgument(
            'frequency',
            default_value = '40',
            description = 'Frequency of IMU Readings'
        ),

        Node(
            package='imu_driver',
            executable='imu_driver',
            name='imu_driver_data',
            output='screen',
            parameters = [{'port' : LaunchConfiguration('port'), 
                           'baudrate' : LaunchConfiguration('baudrate'), 
                           'frequency' : LaunchConfiguration('frequency')
            }]
        )
    ])
