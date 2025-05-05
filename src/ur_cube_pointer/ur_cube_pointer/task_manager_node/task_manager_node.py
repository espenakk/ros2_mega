#!/usr/bin/env python3
"""
ROS2 Node: task_manager_node

Beskrivelse:
-------------
Denne noden styrer sekvensen der roboten peker på fargede kuber i definert rekkefølge.
Den fungerer som systemets "task manager" og koordinerer kamera og robot.

Funksjonalitet:
---------------
- Venter på at brukeren sender kommandoen "start" på topic `/start_sequence`
- Flytter roboten til en hjemposisjon før oppstart og mellom hver kube
- Prøver å peke på hver kube (rød, gul, blå) basert på posisjoner mottatt fra kamera-node
- Ved manglende kube forsøker roboten flere kameravinkler (fallback)
- Bruker en service for å kontrollere robotbevegelse (`move_to_pose`)
- Logger status og feil til terminal

ROS2-parametre:
---------------
- `color_order`: Liste over farger i ønsket rekkefølge (default: ['red', 'yellow', 'blue'])
- `fallback_camera_views`: Liste over navngitte fallback-posisjoner (eks: ['camera_view_1', 'camera_view_2'])
- `fallback_wait_time`: Tid (sekunder) å vente etter hvert kameraflytt for kube-deteksjon (default: 5.0)
- `robot_move_timeout`: Maks ventetid (sekunder) på respons fra robotens servicekall (default: 5.0)

Avhengigheter til andre noder:
------------------------------
Kamera-node (Person 1):
  - Publiserer `geometry_msgs/PoseStamped` på:
      /detected_cubes/red
      /detected_cubes/yellow
      /detected_cubes/blue
  - Posene må være i robotens base-frame og inkludere høyde (pose.position.z)

Robotkontroller-node (Person 2):
  - Tilbyr ROS2-service: `move_to_pose` av type `MoveToPose`
  - Servicen støtter:
      - `target_pose`: PoseStamped (for kube-posisjoner)
      - `target_name`: string (eks: "home", "camera_view_1")
  - Returnerer: `bool success`

Kommando for å starte sekvens:
------------------------------
    ros2 topic pub /start_sequence std_msgs/String "data: 'start'"

Eksempel på Launch Description:
-------------------------------
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ur_cube_pointer',
            executable='task_manager_node.py',
            name='task_manager_node',
            parameters=[
                {'color_order': ['red', 'yellow', 'blue']},
                {'fallback_camera_views': ['camera_view_1', 'camera_view_2']},
                {'fallback_wait_time': 5.0},
                {'robot_move_timeout': 5.0},
            ]
        )
    ])
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from ur_cube_pointer.srv import MoveToPose  # ← Tilpasses faktisk plassering

class TaskManagerNode(Node):
    def __init__(self):
        super().__init__('task_manager_node')

        # === ROS2-parametre ===
        self.declare_parameter('color_order', ['red', 'yellow', 'blue'])
        self.declare_parameter('fallback_camera_views', ['camera_view_1', 'camera_view_2'])
        self.declare_parameter('fallback_wait_time', 5.0)
        self.declare_parameter('robot_move_timeout', 5.0)

        self.color_order = self.get_parameter('color_order').get_parameter_value().string_array_value
        self.fallback_views = self.get_parameter('fallback_camera_views').get_parameter_value().string_array_value
        self.fallback_wait = self.get_parameter('fallback_wait_time').get_parameter_value().double_value
        self.move_timeout = self.get_parameter('robot_move_timeout').get_parameter_value().double_value

        self.cube_poses = {}
        self.sequence_started = False

        # === Abonnenter for kube-posisjoner ===
        self.subscribers = {}
        for color in self.color_order:
            topic = f'/detected_cubes/{color}'
            self.subscribers[color] = self.create_subscription(
                PoseStamped,
                topic,
                lambda msg, c=color: self.cube_callback(msg, c),
                10
            )

        # === Abonnent for manuell start ===
        self.command_sub = self.create_subscription(
            String,
            '/start_sequence',
            self.start_command_callback,
            10
        )

        # === Service-klient til robotkontroller ===
        self.move_client = self.create_client(MoveToPose, 'move_to_pose')
        while not self.move_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Venter på robotens move_to_pose-tjeneste...')

    def cube_callback(self, msg, color):
        self.get_logger().info(f"[Vision] Posisjon for {color} kube mottatt.")
        self.cube_poses[color] = msg

    def start_command_callback(self, msg):
        if msg.data.lower() == 'start' and not self.sequence_started:
            self.sequence_started = True
            self.get_logger().info("Starter pekesekvens.")
            self.start_sequence()

    def start_sequence(self):
        self.move_to_named_position('home')

        for color in self.color_order:
            self.get_logger().info(f"Behandler farge: {color}")

            if color in self.cube_poses:
                if not self.move_to_cube(self.cube_poses[color], color):
                    self.get_logger().error(f"Feil ved flytting til {color}-kube.")
                    return
            else:
                self.get_logger().warn(f"{color}-kube ikke oppdaget. Starter fallback...")
                if not self.run_fallback(color):
                    self.get_logger().error(f"{color}-kube ble ikke funnet etter fallback.")
                    return

            self.move_to_named_position('home')  # Gå tilbake til home mellom kuber

        self.get_logger().info("Sekvens fullført.")

    def move_to_named_position(self, name):
        """Flytter roboten til en forhåndsdefinert posisjon (f.eks. home, camera_view_X)"""
        request = MoveToPose.Request()
        request.target_name = name

        future = self.move_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=self.move_timeout)

        if future.done() and future.result().success:
            self.get_logger().info(f"Flyttet til posisjon: {name}")
            return True
        else:
            self.get_logger().error(f"Kunne ikke flytte til {name}")
            return False

    def move_to_cube(self, pose, color):
        """Flytter roboten til en kube-posisjon"""
        request = MoveToPose.Request()
        request.target_pose = pose

        future = self.move_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=self.move_timeout)

        if future.done() and future.result().success:
            self.get_logger().info(f"Flyttet til {color}-kube.")
            return True
        else:
            self.get_logger().error(f"Feil under bevegelse til {color}-kube.")
            return False

    def run_fallback(self, color):
        """Fallback-strategi: Prøver alle kameravisninger én etter én"""
        for view in self.fallback_views:
            self.get_logger().info(f"Fallback: flytter til {view} for ny visning.")
            if not self.move_to_named_position(view):
                continue  # Prøv neste fallback-visning

            self.get_logger().info(f"Venter {self.fallback_wait:.1f} sekunder på kube-deteksjon...")
            rclpy.spin_once(self, timeout_sec=self.fallback_wait)

            if color in self.cube_poses:
                self.get_logger().info(f"{color}-kube oppdaget i fallback.")
                return self.move_to_cube(self.cube_poses[color], color)

        return False


def main(args=None):
    rclpy.init(args=args)
    node = TaskManagerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
