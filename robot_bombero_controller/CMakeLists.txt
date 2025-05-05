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

include_directories(include)

# Controlador mecanum
add_library(mecanum_controller SHARED
  src/mecanum_controller.cpp
)
ament_target_dependencies(mecanum_controller
  rclcpp controller_interface hardware_interface pluginlib
)
pluginlib_export_plugin_description_file(controller_interface pluginlib_mecanum.xml)

# Controlador del brazo servo
add_library(arm_servo_controller SHARED
  src/arm_servo_controller.cpp
)
ament_target_dependencies(arm_servo_controller
  rclcpp controller_interface hardware_interface pluginlib
)
pluginlib_export_plugin_description_file(controller_interface pluginlib_arm.xml)

# Instalación
install(TARGETS
  mecanum_controller
  arm_servo_controller
  DESTINATION lib
)

install(DIRECTORY include/
  DESTINATION include/
)

install(FILES
  pluginlib_mecanum.xml
  pluginlib_arm.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
