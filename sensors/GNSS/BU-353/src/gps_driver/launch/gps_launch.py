from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'port',
            default_value = '/dev/ttyACM0',
            description = 'Serial'
        ),

        Node(
            package='gps_driver',
            executable='gps_driver',
            name='gps_driver_data',
            output='screen',
            parameters = [{'port' : LaunchConfiguration('port')}]
        ),
    ])
