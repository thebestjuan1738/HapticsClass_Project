from setuptools import find_packages, setup

package_name = 'user_interface'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jensen Coombs',
    maintainer_email='jensencoombs@gmail.com',
    description='User Interface Visual Updates',
    license='Apache License 2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'interface_node = user_interface.interface_loop:main'
        ],
    },
)
