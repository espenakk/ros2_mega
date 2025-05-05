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

def generate_launch_description():
    ld = LaunchDescription()

    # TODO: Add parameter loading from camera_params.yaml

    camera_node = Node(
        package='ur_cube_pointer',
        executable='camera_node',
        name='camera_node',
        output='screen',
        # parameters=[...] # Load parameters here
    )

    # TODO: Add image view or RViz node for visualization

    ld.add_action(camera_node)

    return ld
