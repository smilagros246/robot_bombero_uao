cmake_minimum_required(VERSION 3.8)
project(robot_bombero_controller)

# Opciones de compilación
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencias
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

include_directories(include)

# Biblioteca compartida única con ambos controladores
add_library(${PROJECT_NAME} SHARED
  src/mecanum_controller.cpp
  src/arm_servo_controller.cpp
  src/sensor_state_publisher_controller.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  controller_interface
  hardware_interface
  pluginlib
  realtime_tools
  sensor_msgs
  std_msgs
  geometry_msgs
  nav_msgs
  tf2_ros
)

pluginlib_export_plugin_description_file(controller_interface plugins.xml)

# Instalación
install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib
)

install(DIRECTORY include/
  DESTINATION include/
)

install(FILES
  plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

ament_package()
