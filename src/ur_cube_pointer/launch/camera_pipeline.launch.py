# Copyright 2025 Rune Espenakk
#
# Licensed under the MIT License;
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://opensource.org/licenses/MIT
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
from launch_ros.actions import Node
import os

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get path to the YAML file
    config_path = os.path.join(
        get_package_share_directory('ur_cube_pointer'),
        'config',
        'color_params.yaml'
    )

    return LaunchDescription([
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='usb_cam',
            parameters=[],
            remappings=[],
            output='screen',
            arguments=['--ros-args', '-p', 'video_device:=/dev/video0']  # Specify the video device here
        ),

        Node(
            package='color_detection',
            executable='color_detection_node',
            name='color_detector',
            parameters=[config_path],
            output='screen'
        )
    ])
