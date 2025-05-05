cmake_minimum_required(VERSION 3.8)
project(robot_bombero_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencias
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)

# Archivos de cabecera
include_directories(
  include
)

# Nodo de hardware
add_library(${PROJECT_NAME}
  src/firebot_hardware_interface.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  hardware_interface
  pluginlib
  rclcpp
)

# Registrar como plugin
pluginlib_export_plugin_description_file(hardware_interface pluginlib.xml)

install(
  DIRECTORY include/
  DESTINATION include/
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(FILES pluginlib.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(hardware_interface pluginlib rclcpp)
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

ament_package()
