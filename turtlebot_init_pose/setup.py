from setuptools import setup, find_packages
import os
import glob

package_name = 'turtlebot_init_pose'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        # Install package.xml
        (os.path.join('share', package_name), ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='A node to initialize turtlebot3 in Gazebo and set initial pose',
    license='License declaration',
    entry_points={
        'console_scripts': [
            'initial_pose_setter = turtlebot_init_pose.initial_pose_setter:main',
        ],
    },
)
