import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    robot_description_path = os.path.join(
        get_package_share_directory('robot_bombero_description'),
        'urdf', 'robot_bombero.urdf.xacro'
    )

    declare_robot_description = DeclareLaunchArgument(
        'robot_description', default_value=robot_description_path
    )

    # Nodo joy_node para leer el gamepad
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        parameters=[{'dev': '/dev/input/js0'}]  # O cambiar según el dispositivo real
    )

    # Nodo de teleoperación
    joy_teleop_node = Node(
        package='robot_bombero_teleop',
        executable='teleop_node',
        name='teleop_node',
        output='screen'
    )

    # Nodo de visión (cámara RGB + térmica)
    vision_node = Node(
        package='robot_bombero_vision',
        executable='vision_node',
        name='vision_node',
        output='screen'
    )

    # RViz para visualización
    rviz_config = os.path.join(
        get_package_share_directory('robot_bombero_description'),
        'rviz', 'view_robot.rviz'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )

    return LaunchDescription([
        declare_robot_description,
        joy_node,
        joy_teleop_node,
        vision_node,
        rviz_node
    ])
