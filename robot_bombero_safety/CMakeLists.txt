cmake_minimum_required(VERSION 3.5)
project(robot_bombero_safety)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(include)

add_executable(safety_node src/safety_node.cpp)
ament_target_dependencies(safety_node
  rclcpp
  geometry_msgs
  sensor_msgs
  std_msgs
)

install(TARGETS safety_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
