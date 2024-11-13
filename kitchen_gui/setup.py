from setuptools import setup
import os
from glob import glob

package_name = 'kitchen_gui'

setup(
    name=package_name,
    version='0.0.0',
    packages=['kitchen_gui'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 메시지 및 서비스 파일 추가
        ('share/' + package_name + '/srv', glob('srv/*.srv')),
        ('share/' + package_name + '/msg', glob('msg/*.msg')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rokey',
    maintainer_email='ssm06081@naver.com',
    description='Kitchen GUI package for ROS 2 Humble',
    license='MIT',
    entry_points={
        'console_scripts': [
            'kitchen_gui_node = kitchen_gui.kitchen_gui:main'
        ],
    },
)
