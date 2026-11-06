from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'sensor_bridge'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Haptics Project',
    maintainer_email='ca38743@eid.utexas.edu',
    description='Arduino sensor serial bridge to ROS 2 topics',
    license='Apache License 2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'sensor_node = sensor_bridge.sensor_node:main',
        ],
    },
)
