import os
from setuptools import find_packages, setup

package_name = 'robot_services'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 서비스 파일 포함
        (os.path.join('share', package_name, 'srv'), ['srv/SendOrder.srv', 'srv/CallStaff.srv']),
    ],
    install_requires=['setuptools', 'rclpy', 'std_msgs'],
    zip_safe=True,
    maintainer='rokey',
    maintainer_email='100rudtjs@naver.com',
    description='공통 서비스 정의 패키지',
    license='Apache License 2.0',
    # tests_require=['pytest'], 
    entry_points={
        'console_scripts': [
            'kitchen_service = robot_services.kitchen_service:main',  # 스크립트 등록
        ],
    },
)
