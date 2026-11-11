from setuptools import find_packages, setup

package_name = 'save_data'

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
    maintainer='jensen',
    maintainer_email='jensen.coombs@my.utexas.edu',
    description='Saves data to a sqlite database',
    license='Apache License 2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'database_node = save_data.database_loop:main'
        ],
    },
)
