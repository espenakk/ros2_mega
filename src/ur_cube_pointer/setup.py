import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'ur_cube_pointer'

setup(
    name=package_name,
    version='0.0.0',
    packages=[
        package_name,
        f'{package_name}.camera_node',
        f'{package_name}.robot_controller_node',
        f'{package_name}.task_manager_node',
    ],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
        (os.path.join('share', package_name, 'rviz'), glob(os.path.join('rviz', '*.rviz'))),
        (os.path.join('share', package_name, 'urdf'), glob(os.path.join('urdf', '*.urdf'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rune',
    maintainer_email='143501123+espenakk@users.noreply.github.com',
    description='ROS2 package to detect colored cubes and point at them with a UR robot.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_node = ur_cube_pointer.camera_node.camera_node:main',
            'cube_detection = ur_cube_pointer.camera_node.cube_detection:main',
            'robot_controller_node = ur_cube_pointer.robot_controller_node.robot_controller_node:main',
            'move_to_position = ur_cube_pointer.robot_controller_node.move_to_position:main',
            'task_manager_node = ur_cube_pointer.task_manager_node.task_manager_node:main',
        ],
    },
)
