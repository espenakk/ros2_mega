from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from moveit_configs_utils import MoveItConfigsBuilder  # Recommended way to load robot description


def generate_launch_description():
    # Node for our robot controller
    robot_controller_node = Node(
        package="robot_controller",
        executable="robot_controller_node",
        name="robot_controller_node",
        output="screen",
        parameters=[
            {"use_sim_time": LaunchConfiguration("use_sim_time", default="true")}
            # Set to true if using Gazebo with /clock
        ],
    )

    return LaunchDescription([
        robot_controller_node,
        DeclareLaunchArgument(  # Add use_sim_time argument
            "use_sim_time",
            default_value="true",
            description="Use simulation (Gazebo) clock if true"
        ),
    ])