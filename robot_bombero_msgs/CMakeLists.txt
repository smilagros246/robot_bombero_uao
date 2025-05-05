cmake_minimum_required(VERSION 3.5)
project(robot_bombero_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/FireStatus.msg"
  "msg/ThermalData.msg"
  "srv/ActivatePump.srv"
)

ament_package()
