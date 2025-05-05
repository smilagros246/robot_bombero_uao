cmake_minimum_required(VERSION 3.5)
project(robot_bombero_uao)

# Para ROS 2, asegúrate de incluir los módulos de ROS 2 y otros paquetes
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# Si el paquete tiene dependencias adicionales, añádelas aquí
# find_package(DEPENDENCIA REQUIRED)


ament_package()
