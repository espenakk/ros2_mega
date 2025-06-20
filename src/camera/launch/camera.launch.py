import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Path to the parameters file for camera node
    camera_node_params_file = os.path.join(
        get_package_share_directory('camera'),
        'config',
        'camera_params.yaml'
    )

    # Define argument for which video device we want to use
    video_device_arg = DeclareLaunchArgument(
        'video_device',
        default_value='/dev/video2',
        description='USB camera device path (e.g., /dev/video0)'
    )

    usb_cam_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='usb_cam',
        parameters=[{
            'video_device': LaunchConfiguration('video_device'),
            'pixel_format': 'yuyv2rgb',
            # 'image_width': 640,
            # 'image_height': 480,
            # 'framerate': 30.0,
        }]
        # Publish to /image_raw and /camera_info
    )

    camera_node_cpp_params_arg = DeclareLaunchArgument(
        'camera_params_file',
        default_value=camera_node_params_file,
        description='Full path to the camera_node parameters file'
    )

    camera_node = Node(
        package='camera',
        executable='camera_node',
        name='camera_node',
        output='screen',
        parameters=[LaunchConfiguration('camera_params_file')]
    )

    return LaunchDescription([
        video_device_arg,
        usb_cam_node,
        camera_node_cpp_params_arg,
        camera_node
    ])