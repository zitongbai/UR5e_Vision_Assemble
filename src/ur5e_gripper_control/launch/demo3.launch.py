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

    left_target_pose_list = os.path.join(
        get_package_share_directory('ur5e_gripper_control'),
        'config', 
        'target_pose_list.yaml'
    )

    return LaunchDescription([
        Node(
            package='ur5e_gripper_control',
            executable='demo3',
            name='dem3_node',
            parameters=[{
                    "use_sim_time":True,
                },
                robot_description_kinematics, 
                left_target_pose_list
            ], 
            output='screen'
        ),
    ])