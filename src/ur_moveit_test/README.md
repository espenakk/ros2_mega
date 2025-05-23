
# ur3_moveit_controller_pkg/README.md

## UR3 MoveIt Controller Package

This package provides a simple C++ ROS 2 node (`robot_controller_node`) to control a UR3 robot arm using MoveIt 2.

### Prerequisites

1.  **ROS 2 Installation:** Ensure you have ROS 2 (Humble or newer recommended) installed.
2.  **MoveIt 2 Installation:** Install MoveIt 2 and its dependencies.
    ```bash
    sudo apt update
    sudo apt install ros-<your_ros_distro>-moveit
    ```
3.  **UR Robot Packages:** You need the Universal Robots ROS 2 driver and MoveIt configurations.
    * Typically, these are `ur_robot_driver`, `ur_description`, `ur_moveit_config`, etc.
    * You can install them via apt:
        ```bash
        sudo apt install ros-<your_ros_distro>-universal-robots-ros2-driver
        sudo apt install ros-<your_ros_distro>-ur-moveit-config
        ```
    * Or build them from source if you need a specific version.

### Package Contents

* `src/robot_controller_node.cpp`: The C++ node that uses `MoveGroupInterface` to plan and execute motions.
* `launch/robot_controller.launch.py`: A launch file to start the `robot_controller_node`.
* `package.xml`: Package manifest.
* `CMakeLists.txt`: CMake build instructions.

### Build Instructions

1.  Navigate to your ROS 2 workspace `src` directory:
    ```bash
    cd ~/your_ros2_ws/src
    ```
2.  Clone or copy this package into the `src` directory. If you manually create the files, your directory structure should be:
    ```
    your_ros2_ws/
    └── src/
        └── ur3_moveit_controller_pkg/
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
    cd ~/your_ros2_ws
    colcon build --packages-select ur3_moveit_controller_pkg
    ```
4.  Source your workspace:
    ```bash
    source install/setup.bash
    ```

### Running the Controller

**IMPORTANT:** This controller node is a client to the MoveIt `move_group` server. You must first launch the main MoveIt components for your UR3 robot.

1.  **Start MoveIt for UR3:**
    Open a new terminal and launch the UR MoveIt configuration. Replace `ur3` with your specific UR type if different. Set `use_fake_hardware:=true` for simulation or `false` if connecting to a real robot (ensure `ur_robot_driver` is configured and running for a real robot).
    ```bash
    # For simulation with fake hardware:
    ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur3 use_fake_hardware:=true launch_rviz:=true
    
    # If you have a Gazebo simulation running:
    # ros2 launch ur_simulation_gazebo ur_sim_control.launch.py ur_type:=ur3
    # And then in another terminal (ensure use_sim_time is true in ur_moveit.launch.py or pass it as an argument):
    # ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur3 launch_rviz:=true use_sim_time:=true
    ```
    Wait for RViz to appear and for the `move_group` node to fully initialize (you should see messages like "All is well! Everyone is happy! You can start planning now.").

2.  **Run the Robot Controller Node:**
    Open another terminal, source your workspace, and run the launch file for this package:
    ```bash
    cd ~/your_ros2_ws
    source install/setup.bash
    ros2 launch ur3_moveit_controller_pkg robot_controller.launch.py
    ```

    You should see log messages from `robot_controller_node`. After a few seconds (due to the timer in the C++ code), it will attempt to plan and move the robot to the predefined pose.

### Customization

* **Planning Group:** The `PLANNING_GROUP_ARM` constant in `robot_controller_node.cpp` is set to `"ur_manipulator"`. If your UR3 SRDF uses a different group name (e.g., `"manipulator"`), you **must** change this value. You can also pass it as an argument in the launch file.
* **Target Pose/Joints:** Modify the `target_pose` or `target_joint_values` in `robot_controller_node.cpp` to send the robot to different goals.
* **Real Robot vs. Simulation:**
    * Ensure `use_sim_time` is set correctly in your launch files if using Gazebo.
    * For a real robot, ensure the `ur_robot_driver` is running and configured to connect to your robot's IP address.

### Troubleshooting

* **"Could not find planning group..."**: Double-check the `PLANNING_GROUP_ARM` in the C++ code and the `planning_group` argument in the launch file against your robot's SRDF file.
* **"Unable to connect to move_group action server..."**: Ensure `move_group` is running (from `ur_moveit.launch.py` or similar) before launching `robot_controller_node`.
* **Planning Fails**: The target pose might be unreachable, in collision, or outside joint limits. Try a simpler, known-good pose. Check RViz for collision warnings.
* **Colcon Build Errors**: Check `package.xml` and `CMakeLists.txt` for correct dependencies and syntax. Ensure all dependent ROS packages are installed.
