from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os
import shutil

def generate_launch_description():
    # Get path to xacro file and world
    pkg_path = get_package_share_directory('4wd_robot_sim')
    urdf_path = os.path.join(pkg_path, 'urdf', '4wd_robot.xacro')
    world_path = os.path.join(pkg_path, 'worlds', 'custom_world.sdf')

    # Get xacro executable
    xacro_path = shutil.which('xacro')

    return LaunchDescription([

        # 1. Launch Gazebo with the custom world
        ExecuteProcess(
            cmd=['gazebo', '--verbose', world_path, '-s', 'libgazebo_ros_factory.so'],
            output='screen'
        ),

        # 2. Joint State Publisher (needed by robot_state_publisher)
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen'
        ),

        # 3. Robot State Publisher with xacro
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': ParameterValue(
                    Command([xacro_path, ' ', urdf_path]),
                    value_type=str
                )
            }]
        ),

        # 4. Spawn robot in Gazebo
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
        ),

        # 5. Teleop Twist Keyboard node (remapped to /teleop_cmd)
        Node(
            package='teleop_twist_keyboard',
            executable='teleop_twist_keyboard',
            name='teleop_keyboard',
            output='screen',
            remappings=[
                ('/cmd_vel', '/teleop_cmd')
            ]
        ),

        # 6. Obstacle Stop Node (listens to /teleop_cmd, publishes to /cmd_vel)
        Node(
            package='4wd_robot_sim',
            executable='obstacle_stop',
            name='obstacle_stop_node',
            output='screen'
        )
    ])

