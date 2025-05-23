from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from moveit_configs_utils import MoveItConfigsBuilder # Recommended way to load robot description

def generate_launch_description():
    # Declare a launch argument for the planning group name
    # This makes it easier to configure if needed, though "ur_manipulator" is common for UR3
    planning_group_arg = DeclareLaunchArgument(
        "planning_group",
        default_value="ur_manipulator", # or "manipulator"
        description="Name of the MoveIt planning group for the UR arm.",
    )

    # Note: This launch file assumes that the main MoveIt setup
    # (move_group, robot_state_publisher, rviz, controllers)
    # is already launched, for example, by a launch file from ur_moveit_config.
    # Example: ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur3
    #
    # If you want this launch file to also start MoveIt, you would need to include
    # the MoveIt launch files here. For simplicity, we keep it separate.

    # If you need to load robot description parameters (robot_description, robot_description_semantic)
    # because they are not published by another launch file, you can use MoveItConfigsBuilder.
    # This is often needed if you are not launching the main move_group launch file.
    # However, if move_group is already running, it usually provides these parameters.
    
    # For this example, we assume move_group is running and provides parameters.
    # If not, uncomment and configure the following:
    # moveit_config = (
    #     MoveItConfigsBuilder("ur", package_name="ur_moveit_config")
    #     .robot_description(file_path="config/ur.urdf.xacro", mappings={"ur_type": "ur3"}) # Adjust path and ur_type
    #     .robot_description_semantic(file_path="config/ur.srdf.xacro", mappings={"ur_type": "ur3"}) # Adjust
    #     .trajectory_execution(file_path="config/moveit_controllers.yaml")
    #     .planning_pipelines(pipelines=["ompl", "chomp", "pilz_industrial_motion_planner"]) # Add/remove as needed
    #     .to_moveit_configs()
    # )
    
    # Node for our robot controller
    robot_controller_node = Node(
        package="ur_moveit_test",
        executable="robot_controller_node",
        name="robot_controller_node",
        output="screen",
        parameters=[
            {"planning_group": LaunchConfiguration("planning_group")},
            # If you used MoveItConfigsBuilder above, pass its parameters:
            # moveit_config.to_dict(),
            # Required for intra-process communication
            {"use_sim_time": LaunchConfiguration("use_sim_time", default="false")} # Set to true if using Gazebo with /clock
        ],
    )

    return LaunchDescription([
        planning_group_arg,
        robot_controller_node,
        DeclareLaunchArgument( # Add use_sim_time argument
            "use_sim_time",
            default_value="false",
            description="Use simulation (Gazebo) clock if true"
        ),
    ])