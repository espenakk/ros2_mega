# Camera Package

This package is responsible for handling camera functionalities, specifically for detecting colored cubes and their coordinates.

## Overview

The `camera_node` within this package captures images from a camera, processes these images to detect cubes of predefined colors (e.g., red, blue, yellow), and publishes their 3D coordinates.

## Dependencies

*   `rclcpp`
*   `sensor_msgs` (for image messages)
*   `cv_bridge` (to convert ROS image messages to OpenCV images)
*   `libopencv-dev` (OpenCV library for image processing)
*   `geometry_msgs` (for publishing coordinates)
*   `custom_interfaces` (for custom message types like `DetectedCube` and `DetectedCubes`)

## Building

To build this package, navigate to your ROS 2 workspace and run:
```bash
cd ~/ros2_mega # Or your ROS 2 workspace root
colcon build --packages-select camera
```
Then source the workspace:
```bash
source install/setup.bash
```

## Usage

To run the camera node, use the provided launch file:
```bash
ros2 launch camera camera.launch.py
```
This will start the `camera_node`.

### Nodes

*   `camera_node`:
    *   **Functionality**: Captures images, detects colored cubes, and publishes their identities and coordinates.
    *   **Published Topics**:
        *   `/detected_cubes` (`custom_interfaces/msg/DetectedCubes`): Publishes an array of detected cubes, including their color and 3D coordinates.
    *   **Parameters**: Colour thresholds for detection(See `config/camera_params.yaml`).

## Troubleshooting

*   **Node not starting/crashing**:
    *   Ensure the camera is connected and accessible by the system.
    *   Verify that all dependencies are installed correctly.
    *   Check `colcon build` output for errors.
*   **No detections**:
    *   Check camera feed to ensure images are being captured.
    *   Verify color detection parameters/thresholds are correctly set for your environment and cube colors.
    *   Ensure the `custom_interfaces` package is correctly built and sourced.
*   **Incorrect coordinates**:
    *   Verify camera calibration.
    *   Check the coordinate transformation logic in `camera_node.cpp`.
