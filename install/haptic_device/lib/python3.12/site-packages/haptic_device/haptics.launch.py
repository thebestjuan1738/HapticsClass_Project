
import os
from pathlib import Path

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # Libraries needed by your older OpenHaptics SDK.
    compatibility_path = str(
        Path.home() / 'haptic_setup/compat/lib/x86_64-linux-gnu'
    )

    # Preserve the library paths ROS has already configured.
    existing_path = os.environ.get('LD_LIBRARY_PATH', '')
    library_path = compatibility_path

    if existing_path:
        library_path += os.pathsep + existing_path

    # Describe the process ROS should start.
    return LaunchDescription([
        Node(
            package='haptic_device',
            executable='haptic_loop',
            additional_env={
                'LD_LIBRARY_PATH': library_path,
            },
            output='screen',
        ),
    ])