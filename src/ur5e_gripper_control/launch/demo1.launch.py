import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():

    robot_description_kinematics = PathJoinSubstitution(
        [FindPackageShare("dual_ur5e_gripper_moveit_config"), "config", "kinematics.yaml"]
    )

    return LaunchDescription([
        Node(
            package='ur5e_gripper_control',
            # namespace='left_ur5e',
            executable='demo1',
            name='demo1_left_ur5e_node',
            parameters=[{
                "target_position_1": [0.5, -0.3, 0.4],
                "target_position_2": [0.5, -0.3, 0.15],
                "target_grasp_angle": 0.42,
                "target_position_3": [0.5, 0.0, 0.4],
                "use_sim_time":True, 
            },
            robot_description_kinematics
            ]
        ),
    ])