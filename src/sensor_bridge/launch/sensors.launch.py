from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'port', default_value='',
            description='Serial port, e.g. COM4 (Windows) or /dev/ttyACM0 (Linux/WSL). '
                        'Empty = auto-detect the Arduino.'),
        DeclareLaunchArgument(
            'baud', default_value='115200',
            description='Serial speed (ignored by the R4 native USB).'),
        DeclareLaunchArgument(
            'publish_rate_hz', default_value='100.0',
            description='Publish rate. 100 Hz matches haptic_loop; <= 0 publishes every sample.'),
        DeclareLaunchArgument(
            'sample_mode', default_value='latest',
            description="'latest', 'mean' or 'all' (see sensor_node.py)."),
        Node(
            package='sensor_bridge',
            executable='sensor_node',
            parameters=[{
                'port': ParameterValue(LaunchConfiguration('port'), value_type=str),
                'baud': ParameterValue(LaunchConfiguration('baud'), value_type=int),
                'publish_rate_hz': ParameterValue(
                    LaunchConfiguration('publish_rate_hz'), value_type=float),
                'sample_mode': ParameterValue(
                    LaunchConfiguration('sample_mode'), value_type=str),
            }],
            output='screen',
        ),
    ])
