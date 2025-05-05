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

    # TODO: Add parameter loading from YAML files

    camera_node = Node(
        package='ur_cube_pointer',
        executable='camera_node',
        name='camera_node',
        output='screen',
        # parameters=[...] # Load parameters here
    )

    robot_controller_node = Node(
        package='ur_cube_pointer',
        executable='robot_controller_node',
        name='robot_controller_node',
        output='screen',
        # parameters=[...] # Load parameters here
    )

    task_manager_node = Node(
        package='ur_cube_pointer',
        executable='task_manager_node',
        name='task_manager_node',
        output='screen',
        # parameters=[...] # Load parameters here
    )

    temp_cube_detection = Node(
        package='ur_cube_pointer',
        executable='cube_detection',
        name='temp_cube_detection',
        output='screen',
        # parameters=[...] # Load parameters here
    )

    temp_move_to_position = Node(
        package='ur_cube_pointer',
        executable='move_to_position',
        name='temp_move_to_position',
        output='screen',
        # parameters=[...] # Load parameters here
    )

    ld.add_action(camera_node)
    ld.add_action(robot_controller_node)
    ld.add_action(task_manager_node)
    ld.add_action(temp_cube_detection)
    ld.add_action(temp_move_to_position)
    # TODO: Add RViz node if needed

    return ld
