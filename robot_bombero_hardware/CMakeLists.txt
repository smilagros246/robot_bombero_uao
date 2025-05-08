cmake_minimum_required(VERSION 3.8)
project(robot_bombero_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)

# Buscar la librería LibSerial manualmente
find_library(LIBSERIAL_LIB NAMES serial libserial PATHS /usr/lib /usr/local/lib REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

include_directories(
  include
  /usr/include
  /usr/local/include
)

# Crear la librería de hardware
add_library(${PROJECT_NAME}
  src/firebot_hardware_interface.cpp
)

# Enlazar con LibSerial y otras dependencias
target_link_libraries(${PROJECT_NAME}
  ${LIBSERIAL_LIB}
)

ament_target_dependencies(${PROJECT_NAME}
  hardware_interface
  pluginlib
  rclcpp
)

# Pluginlib
pluginlib_export_plugin_description_file(hardware_interface pluginlib.xml)

# Instalación
install(DIRECTORY include/
  DESTINATION include/
)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(FILES pluginlib.xml
  DESTINATION share/${PROJECT_NAME}
)

# Exportar
ament_export_dependencies(hardware_interface pluginlib rclcpp)
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME})

ament_package()
