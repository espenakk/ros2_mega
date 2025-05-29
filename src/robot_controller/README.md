# Robot Controller Package

## Overview

This package provides a C++ ROS 2 node (`robot_controller_node`) to control a Universal Robot (UR) arm, such as a UR3, using MoveIt 2. It is designed to receive target poses or joint configurations and command the robot to those positions. This package acts as a motion execution server, typically for higher-level task planning (e.g., from the `task_manager` package).

The package name in `package.xml` is `robot_controller`.

### Prerequisites

1.  **ROS 2 Installation:** Ensure you have ROS 2 Jazzy installed.
2.  **MoveIt 2 Installation:** Install MoveIt 2 and its dependencies for ROS 2 Jazzy.
    ```bash
    sudo apt update
    sudo apt install ros-jazzy-moveit
    ```
3.  **UR Robot Packages:** You need the Universal Robots ROS 2 driver and MoveIt configurations for ROS 2 Jazzy.
    *   Typically, these are `ur_robot_driver`, `ur_description`, `ur_moveit_config`, etc.
    *   You can install them via apt:
        ```bash
        sudo apt install ros-jazzy-universal-robots-ros2-driver
        sudo apt install ros-jazzy-ur-moveit-config
        ```
    *   Or build them from source if you need a specific version.

### Package Contents

*   `src/robot_controller_node.cpp`: The C++ node that uses `moveit::planning_interface::MoveGroupInterface` to plan and execute motions. It exposes an action server to receive goals.
*   `launch/robot_controller.launch.py`: A launch file to start the `robot_controller_node` and load its parameters.
*   `package.xml`: Package manifest, defines dependencies like `rclcpp`, `moveit_ros_planning_interface`, `geometry_msgs`, `tf2`, `moveit_configs_utils`.
*   `CMakeLists.txt`: CMake build instructions.

### Build Instructions

1.  Navigate to your ROS 2 workspace `src` directory:
    ```bash
    cd ~/ros2_mega/src # Or your ROS 2 workspace root
    ```
2.  Ensure the package directory is named `robot_controller`. If you cloned it with a different name, rename it. Your directory structure should be:
    ```
    ros2_mega/
    └── src/
        └── robot_controller/
            ├── CMakeLists.txt
            ├── package.xml
            ├── README.md
            ├── src/
            │   └── robot_controller_node.cpp
            └── launch/
                └── robot_controller.launch.py
    ```
3.  Navigate back to the root of your workspace and build:
    ```bash
    cd ~/ros2_mega # Or your ROS 2 workspace root
    colcon build --packages-select robot_controller
    ```
4.  Source your workspace:
    ```bash
    source install/setup.bash
    ```

### Running the Controller

**IMPORTANT:** This controller node is a client to the MoveIt `move_group` server. You must first launch the main MoveIt components for your UR robot.

1.  **Start MoveIt for UR Robot:**
    Open a new terminal and launch the UR MoveIt configuration. Replace `ur3` with your specific UR type if different (e.g., `ur5`, `ur10e`).
    Set `use_fake_hardware:=true` for simulation or `false` if connecting to a real robot (ensure `ur_robot_driver` is configured and running for a real robot).
    ```bash
    # For simulation with fake hardware:
    ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur3 use_fake_hardware:=true launch_rviz:=true
    ```
    Wait for RViz to appear and for the `move_group` node to fully initialize (you should see messages like "All is well! Everyone is happy! You can start planning now." in the MoveIt terminal).

2.  **Run the Robot Controller Node:**
    Open another terminal, source your workspace, and run the launch file for this package:
    ```bash
    cd ~/ros2_mega # Or your ROS 2 workspace root
    source install/setup.bash
    ros2 launch robot_controller robot_controller.launch.py
    ```
    The node should start and be ready to receive motion commands (e.g., via an action server from the `task_manager`).

### Node Details (`robot_controller_node`)

*   **Functionality**: Provides an interface to MoveIt for planning and executing robot arm trajectories.
*   **Subscribed Topics**:
    *   Subscribes to topics related to robot state (`/joint_states`, `/tf`, `/tf_static`) indirectly through MoveIt.
*   **Published Topics**:
    *   Publishes status information or feedback related to action execution.
*   **Action Server**:
    *   Provides an action server for motion commands (e.g., a custom `MoveToTarget.action` or standard `control_msgs/action/FollowJointTrajectory`). This is the primary way other nodes (like `task_manager`) interact with it. The specifics of the action (name, type) are defined in the source code and potentially in an `action/` directory if a custom action is used.
*   **Parameters**:
    *   `planning_group_name`: The name of the MoveIt planning group for the arm (e.g., `"ur_manipulator"` or `"manipulator"`). This is crucial and must match the SRDF configuration of your robot.
    *   Other MoveIt related parameters such as planner IDs, planning time, number of planning attempts, and velocity/acceleration scaling factors.
    *   These parameters are loaded via the `robot_controller.launch.py` file.

### Customization

*   **Planning Group:** The planning group name is critical. It's often defined as a parameter in the launch file or hardcoded in `robot_controller_node.cpp`. Ensure it matches your robot's SRDF (e.g., `"ur_manipulator"`).
*   **Target Goals**: The node is designed to receive goals (poses or joint states) from other nodes, typically via a ROS 2 action. The logic for defining these goals resides in the client node (e.g., `task_manager`).
*   **Real Robot vs. Simulation:**
    *   Ensure `use_sim_time` parameter is set correctly in your launch files when using Gazebo or other time-sensitive simulations. This is usually handled by `ros2 launch` if the simulation environment sets it.
    *   For a real robot, ensure the `ur_robot_driver` is running and correctly configured to connect to your robot's IP address.

### Troubleshooting

*   **"Could not find planning group '...'":**
    *   Verify the `planning_group_name` parameter passed to `robot_controller_node` (in `robot_controller.launch.py` or node parameters) matches a group defined in your robot's SRDF file (e.g., `ur3.srdf`). Common names are `"ur_manipulator"`, `"manipulator"`.
*   **"Unable to connect to move_group action server..." or "MoveGroupInterface_Construction_Failure":**
    *   Ensure the MoveIt `move_group` node (launched via `ur_moveit.launch.py` or similar) is running and fully initialized *before* launching `robot_controller_node`.
    *   Check that ROS 2 networking is configured correctly if nodes are on different machines or in different Docker containers.
*   **Planning Fails (MoveIt Error Codes):**
    *   The target pose might be unreachable (outside workspace, kinematically impossible).
    *   The target pose might be in collision with the robot itself or the environment (if a scene is published to MoveIt).
    *   Joint limits might be violated.
    *   Try a simpler, known-good pose. Use RViz with MotionPlanning display to visualize planning attempts and potential issues.
*   **Colcon Build Errors**:
    *   Ensure all dependencies listed in `package.xml` are installed (e.g., `ros-jazzy-moveit-ros-planning-interface`, `ros-jazzy-tf2-eigen`).
    *   Check `CMakeLists.txt` for correct `find_package` calls and linking.
    *   Ensure the package name used in `colcon build --packages-select` is `robot_controller`.

This README uses `robot_controller` as the package name, which aligns with `package.xml`.
