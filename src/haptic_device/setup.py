from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'haptic_device'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('haptics_launch/*.launch.py'),)
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jensen Coombs',
    maintainer_email='jensencoombs@gmail.com',
    description='Haptic Device Sensing and Environment',
    license='Apache License 2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'haptic_node = haptic_device.start_haptics:main',
            'haptic_loop = haptic_device.haptic_loop:main',
        ],
    },
)
