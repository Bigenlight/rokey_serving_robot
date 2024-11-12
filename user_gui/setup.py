from setuptools import setup
import os
from glob import glob

package_name = 'user_gui'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 이미지 파일을 패키지에 포함
        (os.path.join('share', package_name, 'images'), glob('user_gui/images/*'))
    ],
    install_requires=['setuptools', 'rclpy', 'PyQt5'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='Restaurant Robot GUI as a ROS2 node',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'gui_node = user_gui.user_gui:main',  # 변경된 부분
        ],
    },
)
