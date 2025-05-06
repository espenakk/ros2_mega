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


import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from ur_cube_pointer.srv import MoveToPose
from moveit2_interface import MoveIt2  # Du må justere denne importen etter ditt API

class RobotControllerNode(Node):
    def __init__(self):
        super().__init__('robot_controller_node')

        # Initialiser MoveIt2
        self.moveit2 = MoveIt2(node=self, group_name='manipulator')

        # Service for bevegelse
        self.srv = self.create_service(MoveToPose, 'move_to_pose', self.move_to_pose_callback)

    def move_to_pose_callback(self, request, response):
        if request.target_name:
            self.get_logger().info(f"Flytter til navngitt posisjon: {request.target_name}")
            # Må implementere hvordan navngitte posisjoner lastes fra robot_params.yaml
            pose = self.get_named_pose(request.target_name)
            if pose:
                self.moveit2.move_to_pose(pose)
                response.success = True
            else:
                response.success = False
        elif request.target_pose:
            self.get_logger().info("Flytter til gitt pose.")
            self.moveit2.move_to_pose(request.target_pose)
            response.success = True
        else:
            self.get_logger().error("Ingen gyldig målposisjon mottatt.")
            response.success = False
        return response

    def get_named_pose(self, name):
        # TODO: Last navngitte posisjoner fra YAML-filen
        return None

def main(args=None):
    rclpy.init(args=args)
    node = RobotControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
