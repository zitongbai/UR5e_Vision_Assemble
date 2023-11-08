from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ur5e_gripper_control',
            # namespace='left_ur5e',
            executable='demo1',
            name='demo1_left_ur5e_node',
            parameters=[{
                "target_position_1": [0.5, -0.3, 0.4],
                "target_position_2": [0.5, -0.3, 0.2],
            }]
        ),
    ])