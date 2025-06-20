cmake_minimum_required(VERSION 3.8)
project(camera)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(custom_interfaces REQUIRED)
find_package(std_msgs REQUIRED)

# Camera node
add_executable(camera_node src/camera_node.cpp)
target_include_directories(camera_node PRIVATE
  ${OpenCV_INCLUDE_DIRS}
)
ament_target_dependencies(camera_node
  rclcpp
  sensor_msgs
  cv_bridge
  geometry_msgs
  custom_interfaces
  OpenCV
)

# Install targets
install(TARGETS
  camera_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch and config files
install(
  DIRECTORY
    launch
    config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()