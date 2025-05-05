cmake_minimum_required(VERSION 3.8)
project(robot_bombero_tests)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(ament_lint_auto REQUIRED)

ament_lint_auto_find_test_dependencies()

if(BUILD_TESTING)
  # Esto asume que usarás pytest por convención
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_urdf_xacro test/test_urdf_xacro.py)
  # ament_add_pytest_test(test_pump_activation test/test_pump_activation.py)
  # ament_add_pytest_test(test_fire_status test/test_fire_status.py)
endif()

ament_package()
