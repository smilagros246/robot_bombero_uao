import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command

def generate_launch_description():
    urdf_path = os.path.join(
        get_package_share_directory('robot_bombero_description'),
        'urdf',
        'robot.urdf.xacro'
    )

    robot_description = {'robot_description': Command(['xacro ', urdf_path])}

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description],
        output='screen'
    )

    joy_node = Node(
        package='joy',
        executable='joy_node',
        parameters=[{'dev': '/dev/input/js1'}],
        output='screen'
    )

    joy_teleop_node = Node(
        package='robot_bombero_teleop',
        executable='teleop_node',
        output='screen'
    )
    
    safety_node = Node(
        package='robot_bombero_safety',
        executable='safety_node',
        output='screen'
    )

    rviz_config = os.path.join(
        get_package_share_directory('robot_bombero_description'),
        'rviz',
        'view_robot.rviz'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )
    
     # Nodo de visión (cámara RGB + térmica)
    vision_node = Node(
        package='robot_bombero_vision',
        executable='vision_node',
        name='vision_node',
        output='screen'
    )


    return LaunchDescription([
        robot_state_publisher_node,
        joy_node,
        joy_teleop_node,
        safety_node,
        vision_node,
        rviz_node
    ])

    
    
    