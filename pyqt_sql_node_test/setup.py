from setuptools import setup

package_name = 'pyqt_sql_node_test'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'PyQt5'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='PyQt5 GUI node for ROS2',
    license='Apache License 2.0',
    #tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gui_node = pyqt_sql_node_test.gui_node:main',
        ],
    },
)
