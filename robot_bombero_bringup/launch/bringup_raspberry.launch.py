from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Argumento opcional para activar RViz
    declare_gui_arg = DeclareLaunchArgument(
        "gui", default_value="false", description="Lanza RViz2 con configuración predeterminada."
    )

    # Ruta al xacro
    urdf_path = PathJoinSubstitution([
        FindPackageShare('robot_bombero_description'),
        'urdf',
        'robot.urdf.xacro'
    ])

    robot_description = {
        'robot_description': Command(['xacro ', urdf_path])
    }

    # Configuración del robot
    controller_config = PathJoinSubstitution([
        FindPackageShare('robot_bombero_bringup'),
        'config',
        'controllers.yaml'
    ])

    hardware_config = PathJoinSubstitution([
        FindPackageShare('robot_bombero_hardware'),
        'config',
        'firebot_hardware.yaml'
    ])

    # # RViz opcional
    # rviz_config_file = PathJoinSubstitution([
    #     FindPackageShare('robot_bombero_description'),
    #     'rviz',
    #     'view_robot.rviz'
    # ])

    # gui = LaunchConfiguration("gui")
    # rviz_node = Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     name='rviz2',
    #     arguments=['-d', rviz_config_file],
    #     condition=IfCondition(gui),
    #     output='screen'
    # )

    # Publicador de TF
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    # Nodo ros2_control
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        output='screen',
        parameters=[robot_description, hardware_config, controller_config]
    )

    # # Controlador personalizado (tu nodo C++)
    # firebot_control_node = Node(
    #         package='robot_bombero_control_node',
    #         executable='firebot_control_node',
    #         name='firebot_control_node',
    #         output='screen'
    #     )

    # Spawners de controladores
    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    mecanum_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['mecanum_controller', '--controller-manager', '/controller_manager'],
        output='screen'
    )
    
    sensor_state_pub_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['sensor_state_publisher_controller', '--controller-manager', '/controller_manager'],
        output='screen'
    )
    
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    # # Lanza RViz después del spawner de arm_controller
    # delay_rviz = RegisterEventHandler(
    #     OnProcessExit(
    #         target_action=arm_controller_spawner,
    #         on_exit=[rviz_node]
    #     )
    # )

    return LaunchDescription([
        declare_gui_arg,
        robot_state_publisher_node,
        ros2_control_node,
        # firebot_control_node,
        arm_controller_spawner,
        mecanum_controller_spawner,
        sensor_state_pub_spawner,
        joint_state_broadcaster_spawner
        # delay_rviz
    ])
