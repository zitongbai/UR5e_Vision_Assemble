import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from launch.substitutions import PathJoinSubstitution
from math import pi

def generate_launch_description():

    robot_description_kinematics = PathJoinSubstitution(
        [FindPackageShare("dual_ur5e_gripper_moveit_config"), "config", "kinematics.yaml"]
    )

    return LaunchDescription([
        Node(
            package='ur5e_gripper_control',
            executable='demo2',
            name='demo2_left_ur5e_node',
            parameters=[{
                "target_pose_1": [0.5, -0.3, 0.4, 0, pi, pi/2],
                "target_pose_2": [0.5, -0.3, 0.15, 0, pi, pi/2],
                "target_grasp_angle": 0.38,
                "target_pose_3": [0.5, -0.3, 0.4, pi/2, -pi/2, 0],
                "target_pose_4": [0.5, -0.33, 0.4, pi/2, -pi/2, 0],
                "use_sim_time":True, 
                "which_arm": "left",
            },
            robot_description_kinematics
            ]
        ),
        Node(
            package='ur5e_gripper_control',
            executable='demo2',
            name='demo2_right_ur5e_node',
            parameters=[{
                "target_pose_1": [0.5, 0.3, 0.4, 0, pi, pi/2],
                "target_pose_2": [0.5, 0.3, 0.15, 0, pi, pi/2],
                "target_grasp_angle": 0.38,
                "target_pose_3": [0.5, 0.3, 0.4, -pi/2, pi/2, 0],
                "target_pose_4": [0.5, 0.33, 0.4, -pi/2, pi/2, 0],
                "use_sim_time":True, 
                "which_arm": "right",
            },
            robot_description_kinematics
            ]
        ),
    ])