# Custom Interfaces Package

This package defines custom ROS 2 messages used by other packages in the `ros2_mega` workspace, specifically for representing detected objects (cubes).

## Overview

This package provides message definitions for:
*   `DetectedCube.msg`: Represents a single detected cube, including its color and 3D position.
*   `DetectedCubes.msg`: Represents a list of `DetectedCube` messages.

## Interfaces

### Messages

*   `DetectedCube.msg`:
    *   `std_msgs/Header header`: Standard ROS message header (timestamp, frame_id).
    *   `string color`: The detected color of the cube (e.g., red, blue, yellow).
    *   `geometry_msgs/Point position`: The 3D coordinates (x, y, z) of the detected cube.

*   `DetectedCubes.msg`:
    *   `std_msgs/Header header`: Standard ROS message header.
    *   `custom_interfaces/DetectedCube[] cubes`: An array of `DetectedCube` messages.



## Building

This package is typically built as a dependency by other packages. To build it explicitly:
```bash
cd ~/ros2_mega # Or your ROS 2 workspace root
colcon build --packages-select custom_interfaces
```
Then source the workspace:
```bash
source install/setup.bash
```

## Usage

These interfaces are used in the C++ or Python code of other nodes (e.g., `camera_node`, `task_manager_node`). Ensure that any package using these interfaces lists `custom_interfaces` as a dependency in its `package.xml` and `CMakeLists.txt` (for C++) or `setup.py` (for Python).

**Example CMakeLists.txt dependency:**
```cmake
# Find the custom_interfaces package
find_package(custom_interfaces REQUIRED)

# Add it as a dependency to your target
ament_target_dependencies(your_executable_or_library_target custom_interfaces)
```

**Example package.xml dependency:**
```xml
<depend>custom_interfaces</depend>
```

When using the messages in C++ code, include the headers like so:
```cpp
#include "custom_interfaces/msg/detected_cube.hpp"
#include "custom_interfaces/msg/detected_cubes.hpp"
```
