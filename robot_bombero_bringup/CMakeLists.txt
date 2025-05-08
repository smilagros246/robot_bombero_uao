cmake_minimum_required(VERSION 3.5)
project(robot_bombero_bringup)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(controller_manager REQUIRED)


install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
