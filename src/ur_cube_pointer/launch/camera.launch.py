import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Path to the parameters file for your C++ camera_node
    camera_node_params_file = os.path.join(
        get_package_share_directory('ur_cube_pointer'), # Ensure this is your package name
        'config',
        'camera_params.yaml'
    )

    # --- usb_cam Node Configuration (Simplified) ---
    video_device_arg = DeclareLaunchArgument(
        'video_device',
        default_value='/dev/video2', # You confirmed /dev/video4 works
        description='USB camera device path (e.g., /dev/video0)'
    )

    usb_cam_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='usb_cam', # Default node name for usb_cam
        parameters=[{
            'video_device': LaunchConfiguration('video_device'),
            'pixel_format': 'yuyv2rgb', # Important for BGR8 conversion for OpenCV
            # Using default width/height/framerate from usb_cam,
            # which seemed to be 640x480 @ 30fps and worked.
            # Add them explicitly if you need specific values:
            # 'image_width': 640,
            # 'image_height': 480,
            # 'framerate': 30.0,
        }]
        # No remappings: will publish to /image_raw and /camera_info
    )

    # --- Your Cube Detection C++ Node ---
    camera_node_cpp_params_arg = DeclareLaunchArgument(
        'camera_params_file',
        default_value=camera_node_params_file,
        description='Full path to the C++ camera_node parameters file'
    )

    cube_detector_node = Node(
        package='ur_cube_pointer', # Ensure this is your package name
        executable='camera_node',    # The executable from your C++ code
        name='camera_node',      # The name of your C++ node (as defined in its constructor)
        output='screen',
        parameters=[LaunchConfiguration('camera_params_file')]
    )

    return LaunchDescription([
        video_device_arg,
        usb_cam_node,
        camera_node_cpp_params_arg,
        cube_detector_node
    ])