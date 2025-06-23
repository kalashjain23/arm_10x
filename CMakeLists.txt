cmake_minimum_required(VERSION 3.10)
project(arm_controller_10x)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(cartesian_constant_velocity_controller src/cartesian_constant_velocity_controller.cpp)
ament_target_dependencies(cartesian_constant_velocity_controller rclcpp moveit_ros_planning_interface moveit_msgs geometry_msgs)

install(TARGETS cartesian_constant_velocity_controller
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
