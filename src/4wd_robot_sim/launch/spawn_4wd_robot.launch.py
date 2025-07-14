from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
import os

def generate_launch_description():
    pkg_name = '4wd_robot_sim'

    pkg_share = FindPackageShare(package=pkg_name).find(pkg_name)
    world_file = os.path.join(pkg_share, 'worlds', 'custom_world.sdf')
    urdf_file = os.path.join(pkg_share, 'urdf', '4wd_robot.xacro')

    # Fix: Properly wrap xacro command as string for robot_description
    robot_description = ParameterValue(
        Command([FindExecutable(name='xacro'), ' ', urdf_file]),
        value_type=str
    )

    return LaunchDescription([
        # Start Gazebo with custom world
        ExecuteProcess(
            cmd=[
                'gazebo', world_file, '--verbose',
                '-s', 'libgazebo_ros_factory.so'
            ],
            output='screen'
        ),

        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}]
        ),

        # Spawn robot entity in Gazebo
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_4wd_robot',
            arguments=[
                '-entity', '4wd_robot',
                '-topic', '/robot_description',
                '-x', '0', '-y', '0', '-z', '0.3'
            ],
            output='screen'
        )
    ])
