from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Ruta al xacro
    robot_description_path = os.path.join(
        get_package_share_directory('robot_bombero_description'),
        'urdf', 'robot.urdf.xacro'
    )

    declare_robot_description = DeclareLaunchArgument(
        'robot_description', default_value=robot_description_path
    )

    # robot_state_publisher para publicar las TF desde la Raspberry
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': Command(['xacro ', robot_description_path])
        }]
    )

    # Nodo ros2_control
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        name='ros2_control_node',
        output='screen',
        parameters=[os.path.join(
            get_package_share_directory('robot_bombero_hardware'),
            'config', 'firebot_hardware.yaml'
        )]
    )

    # Nodo de control personalizado
    firebot_control_node = Node(
        package='robot_bombero_control_node',
        executable='firebot_control_node',
        name='firebot_control_node',
        output='screen',
        parameters=[os.path.join(
            get_package_share_directory('robot_bombero_bringup'),
            'config', 'controllers.yaml'
        )],
        remappings=[('/cmd_vel', '/cmd_vel')]
    )

    return LaunchDescription([
        declare_robot_description,
        robot_state_publisher_node,
        ros2_control_node,
        firebot_control_node
    ])
