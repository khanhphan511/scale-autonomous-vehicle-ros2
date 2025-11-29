from setuptools import setup

package_name = 'igvc_lane'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/igvc_q1_lane_lidar.launch.py',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kphan511',
    maintainer_email='you@example.com',
    description='Lane detection and steering for IGVC using OAK + LiDAR',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lane_detector = igvc_lane.lane_detector_node:main',
            'lane_serial_steer = igvc_lane.lane_serial_steer_node:main',
        ],
    },
)
