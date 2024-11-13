from setuptools import setup
import os
from glob import glob

package_name = 'kitchen_gui'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'rclpy', 'PyQt5'],
    zip_safe=True,
    maintainer='rokey',
    maintainer_email='ssm06081@naver.com',
    description='Kitchen GUI package using ROS 2 services',
    license='MIT',
    entry_points={
        'console_scripts': [
            'kitchen_gui_node = kitchen_gui.kitchen_gui:main'
        ],
    },
)
