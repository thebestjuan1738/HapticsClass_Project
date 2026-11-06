from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'port', default_value='',
            description='Serial port, e.g. /dev/ttyACM0 (empty = auto-detect)'),
        DeclareLaunchArgument(
            'baud', default_value='115200'),
        Node(
            package='sensor_bridge',
            executable='sensor_node',
            parameters=[{
                'port': LaunchConfiguration('port'),
                'baud': LaunchConfiguration('baud'),
            }],
            output='screen',
        ),
    ])
