from setuptools import setup

package_name = 'igvc_lidar'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/lidar_obstacle_stop.launch.py',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kphan511',
    maintainer_email='',
    description='LiDAR obstacle avoidance tools for IGVC',
    license='',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lidar_serial_stop_node = igvc_lidar.lidar_serial_stop_node:main',
            'lidar_stop_logic_node = igvc_lidar.lidar_stop_logic_node:main',
        ],
    },
)

