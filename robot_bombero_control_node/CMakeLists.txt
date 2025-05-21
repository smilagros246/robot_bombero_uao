cmake_minimum_required(VERSION 3.8)
project(robot_bombero_control_node)

# Encontrar dependencias necesarias
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(controller_manager REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)             # Añadido sensor_msgs
find_package(robot_bombero_hardware REQUIRED)
find_package(robot_bombero_description REQUIRED)

# Incluir headers si tienes archivos en include/
include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${robot_bombero_hardware_INCLUDE_DIRS}
)

# Ejecutable principal
add_executable(firebot_control_node src/firebot_control_node.cpp)

# Ligar dependencias
ament_target_dependencies(firebot_control_node
  rclcpp
  controller_manager
  hardware_interface
  pluginlib
  std_msgs
  sensor_msgs                          # Añadido sensor_msgs aquí también
  robot_bombero_hardware
)

# Instalar el ejecutable
install(TARGETS firebot_control_node
  DESTINATION lib/${PROJECT_NAME}
)

# Instalar headers si tienes
install(DIRECTORY include/
  DESTINATION include/
)

ament_package()
