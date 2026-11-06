
import os
import sys


def main():
    os.execvp(
        'ros2',
        [
            'ros2',
            'launch',
            'haptic_device',
            'haptics.launch.py',
            *sys.argv[1:],
        ],
    )


if __name__ == '__main__':
    main()