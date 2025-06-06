import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    robot_description_path = os.path.join(
        get_package_share_directory('robot_bombero_description'),
        'urdf', 'robot_bombero.urdf.xacro'
    )

    declare_robot_description = DeclareLaunchArgument(
        'robot_description', default_value=robot_description_path
    )

    # Nodo que publica TFs desde el modelo URDF
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': LaunchConfiguration('robot_description')}]
    )

    # Nodo de ros2_control
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        name='ros2_control_node',
        output='screen',
        parameters=[os.path.join(
            get_package_share_directory('robot_bombero_hardware'),
            'config', 'robot.ros2_control.yaml'
        )]
    )

    # Nodo de control personalizado
    firebot_control_node = Node(
        package='robot_bombero_control_node',
        executable='firebot_control_node',
        name='firebot_control_node',
        output='screen',
        parameters=[os.path.join(
            get_package_share_directory('robot_bombero_control_node'),
            'config', 'control_node.yaml'
        )]
    )

    # Nodo de visión
    vision_node = Node(
        package='robot_bombero_vision',
        executable='vision_node',
        name='vision_node',
        output='screen'
    )

    # Nodo joy_node (lectura del gamepad)
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        parameters=[{'dev': '/dev/input/js0'}]  # Cambia si usas otro dispositivo
    )

    # Nodo de teleoperación
    joy_teleop_node = Node(
        package='robot_bombero_teleop',
        executable='teleop_node',
        name='teleop_node',
        output='screen'
    )

    # RViz
    rviz_config_path = os.path.join(
        get_package_share_directory('robot_bombero_description'),
        'rviz', 'view_robot.rviz'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path]
    )

    return LaunchDescription([
        declare_robot_description,
        robot_state_publisher_node,
        ros2_control_node,
        firebot_control_node,
        vision_node,
        joy_node,
        joy_teleop_node,
        rviz_node
    ])
