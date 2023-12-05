from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='vision',
            executable='obj_seg',
            name='obj_seg',
            output='screen'
        ),
        Node(
            package='vision',
            executable='det_tf',
            name='det_tf',
            output='screen'
        ),
    ])