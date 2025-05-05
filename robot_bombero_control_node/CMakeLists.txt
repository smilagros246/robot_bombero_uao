cmake_minimum_required(VERSION 3.8)
project(robot_bombero_control_node)

# Encontrar dependencias necesarias
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(controller_manager REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)

# Incluir headers si vas a usar archivos en include/
include_directories(include)

# Ejecutable principal
add_executable(firebot_control_node src/firebot_control_node.cpp)

# Ligar dependencias
ament_target_dependencies(firebot_control_node
  rclcpp
  controller_manager
  hardware_interface
  pluginlib
)

# Instalar el ejecutable
install(TARGETS firebot_control_node
  DESTINATION lib/${PROJECT_NAME}
)

# Si usas headers personalizados (futuros), instala include/
install(DIRECTORY include/
  DESTINATION include/
)

ament_package()
