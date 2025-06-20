cmake_minimum_required(VERSION 3.8)
project(robot_controller)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(robot_controller_node src/robot_controller_node.cpp)

ament_target_dependencies(robot_controller_node
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
  tf2
  tf2_geometry_msgs
  tf2_eigen
  std_msgs
)

install(TARGETS
  robot_controller_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()