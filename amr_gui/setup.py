import os
from setuptools import setup, find_packages

package_name = 'amr_gui'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # ROS 2 패키지 리소스
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 이미지 파일 포함
        (os.path.join('share', package_name, 'images'), 
            [os.path.join('images', 'eye-6662_256.gif')]),
    ],
    install_requires=['setuptools', 'rclpy', 'PyQt5'],
    zip_safe=True,
    maintainer='your_name',  # 자신의 이름으로 변경
    maintainer_email='your_email@example.com',  # 자신의 이메일로 변경
    description='AMR 식당 로봇 GUI',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'amr_gui_node = amr_gui.amr_gui_node:main',
            'event_subscribe = amr_gui.goal_monitor_node:main',
        ],
    },
)
