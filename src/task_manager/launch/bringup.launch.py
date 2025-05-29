from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    return LaunchDescription([
        # Include camera.launch.py from camera package
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(
                    get_package_share_directory('camera'),
                    'launch',
                    'camera.launch.py'
                )
            ])
        ),

        # Include robot_controller.launch.py from robot_controller package
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(
                    get_package_share_directory('robot_controller'),
                    'launch',
                    'robot_controller.launch.py'
                )
            ])
        ),

        # Start task_manager_node from task_manager package, with fixed positions loaded as .yaml parameter
        Node(
            package='task_manager',
            executable='task_manager_node',
            name='task_manager',
            parameters=[os.path.join(
                get_package_share_directory('task_manager'),
                'config',
                'fixed_positions.yaml'
            )]
        ),
    ])