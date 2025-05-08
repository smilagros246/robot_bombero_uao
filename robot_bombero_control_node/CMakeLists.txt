cmake_minimum_required(VERSION 3.8)
project(robot_bombero_control_node)

# Encontrar dependencias necesarias
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(controller_manager REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(robot_bombero_controller REQUIRED)
find_package(std_msgs REQUIRED)
find_package(robot_bombero_hardware REQUIRED)




# Incluir headers si vas a usar archivos en include/
include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${robot_bombero_hardware_INCLUDE_DIRS}  
  /usr/include
  /usr/local/include
)
include_directories(${robot_bombero_controller_INCLUDE_DIRS})



# Ejecutable principal
add_executable(firebot_control_node src/firebot_control_node.cpp)

# Ligar dependencias
ament_target_dependencies(firebot_control_node
  rclcpp
  controller_manager
  hardware_interface
  pluginlib
  geometry_msgs 
  std_msgs
  robot_bombero_controller
  robot_bombero_hardware
)

# Instalar el ejecutable
install(TARGETS firebot_control_node
  DESTINATION lib/${PROJECT_NAME}
)

# Si usas headers personalizados (futuros), instala include/
install(DIRECTORY include/
  DESTINATION include/
)


install(TARGETS firebot_control_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
