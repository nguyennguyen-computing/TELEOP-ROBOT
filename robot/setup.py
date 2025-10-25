from setuptools import setup
import os
from glob import glob

package_name = 'teleop_robot'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Web Teleop Robot Team',
    maintainer_email='dev@example.com',
    description='ROS2 robot node for web teleoperation system',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_node = teleop_robot.robot_node:main',
        ],
    },
)