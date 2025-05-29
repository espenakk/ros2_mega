# Task Manager Package

This package is responsible for managing and coordinating tasks for the robot, acting as an intermediary between the `camera` package and the `robot_controller` package.

## Overview

The `task_manager_node` subscribes to information about detected cubes from the `camera_node`. Based on this information, it decides which cube to interact with (e.g., pick up) and sends commands to the `robot_controller` to execute the necessary movements.

## Dependencies

*   `rclcpp`
*   `sensor_msgs` (potentially, if directly handling sensor data, though likely relies on `custom_interfaces`)
*   `geometry_msgs` (for pose/point data)
*   `custom_interfaces` (for `DetectedCubes.msg` and potentially action definitions if used for robot control)
*   `robot_controller` (implicitly, as it sends commands to it)

## Building

To build this package, navigate to your ROS 2 workspace and run:
```bash
cd ~/ros2_mega # Or your ROS 2 workspace root
colcon build --packages-select task_manager
```
Then source the workspace:
```bash
source install/setup.bash
```

## Usage

To run the task manager node:
```bash
ros2 launch task_manager bringup.launch.py 
```

### Nodes

*   `task_manager_node`:
    *   **Functionality**: Receives information about detected objects (cubes) and decides on actions for the robot. It then commands the `robot_controller` to perform these actions.
    *   **Subscribed Topics**:
        *   `/detected_cubes` (`custom_interfaces/msg/DetectedCubes`): Listens for detected cubes from the `camera_node`.
    *   **Published Topics**: Potentially status topics or requests to other nodes (details would be in `task_manager_node.cpp`).
    *   **Services**: May offer services to trigger tasks or report status.
    *   **Actions**: Likely uses an action client to send goals (e.g., `move_to_pose`) to an action server in the `robot_controller` package.
    *   **Parameters**: Could have parameters for task prioritization, target selection logic, etc. (Check `config/` or source code).

## Configuration

Configuration for the task manager, such as decision-making logic parameters or target preferences, might be available in a `config/` directory (e.g., `config/task_params.yaml`) and loaded by its launch file. Refer to `launch/bringup.launch.py` and `src/task_manager_node.cpp` for specifics.

## Interaction with other packages

*   **`camera` package**: Subscribes to `/detected_cubes` topic published by `camera_node` to get information about available cubes.
*   **`robot_controller` package**: Sends commands (likely via ROS 2 actions, e.g., `FollowJointTrajectory` or a custom action for pick-and-place) to the `robot_controller_node` to move the robot arm.
*   **`custom_interfaces` package**: Uses messages like `DetectedCubes` from this package.

## Troubleshooting

*   **Node not receiving cube data**: 
    *   Ensure `camera_node` is running and publishing to `/detected_cubes`.
    *   Verify topic names match using `ros2 topic list` and `ros2 topic echo /detected_cubes`.
*   **Robot not moving or behaving unexpectedly**:
    *   Ensure `robot_controller_node` (and MoveIt, if applicable) is running and healthy.
    *   Check if the `task_manager_node` is correctly sending goals/commands to the `robot_controller`.
    *   Inspect logs from both `task_manager_node` and `robot_controller_node` for errors.
*   **Build errors**: Check dependencies in `package.xml` and `CMakeLists.txt`.
