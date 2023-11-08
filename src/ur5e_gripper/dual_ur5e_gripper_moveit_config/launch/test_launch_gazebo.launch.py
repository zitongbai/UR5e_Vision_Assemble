from ament_index_python.packages import get_package_share_directory
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    world = os.path.join(
        get_package_share_directory('dual_ur5e_gripper_moveit_config'),
        'gazebo',
        'dual_ur5e.world'
    )
    gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose', world, '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'], 
        output='screen'
    )

    return LaunchDescription([
        gazebo
    ])