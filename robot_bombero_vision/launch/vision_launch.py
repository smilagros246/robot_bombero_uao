import launch
from launch import LaunchDescription
from launch.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_bombero_vision',
            executable='vision_node',
            name='vision_node',
            output='screen',
            parameters=[{'image_topic_rgb': '/camera/rgb/image_raw', 'image_topic_thermal': '/camera/thermal/image_raw'}]
        ),
    ])
