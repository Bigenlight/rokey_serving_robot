# turtlebot3_communication/setup.py

from setuptools import setup
import os
from glob import glob

package_name = 'turtlebot3_communication'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='A simple ROS2 package for TurtleBot communication',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'publisher_node = turtlebot3_communication.publisher_node:main',
            'subscriber_node = turtlebot3_communication.subscriber_node:main',
        ],
    },
)
