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

config_path = os.path.join(
        get_package_share_directory('robot_controller'),
        'config',
        'robot_params.yaml'
    )

def generate_launch_description():
    ld = LaunchDescription()

    # This launch file might trigger a service call or action on the robot_controller_node
    # For simplicity, we can start the node itself, assuming it moves home on startup
    # or has a specific service/action for it.

    robot_controller_node = Node(
        package='ur_cube_pointer',
        executable='robot_controller_node',
        name='robot_controller_node',
        output=[config_path],
    )

    ld.add_action(robot_controller_node)

    return ld
