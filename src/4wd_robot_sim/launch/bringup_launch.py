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

        # 1. Launch Gazebo backend
        ExecuteProcess(
            cmd=['gzserver', '--verbose', world_path, '-s', 'libgazebo_ros_factory.so'],
            output='screen'
        ),

        # 2. Launch Gazebo GUI
        ExecuteProcess(
            cmd=['gzclient'],
            output='screen'
        ),

        # 3. Joint State Publisher
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen'
        ),

        # 4. Robot State Publisher with xacro
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

        #  5. Spawn Robot in Gazebo
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

        # 6. Teleop Twist Keyboard
        Node(
            package='teleop_twist_keyboard',
            executable='teleop_twist_keyboard',
            name='teleop_keyboard',
            output='screen',
            remappings=[
                ('/cmd_vel', '/teleop_cmd')
            ]
        ),

        # 7. Obstacle Stop Node
        Node(
            package='4wd_robot_sim',
            executable='obstacle_stop',
            name='obstacle_stop_node',
            output='screen'
        ),

        # 8. Wall Avoidance Node
        Node(
            package='4wd_robot_sim',
            executable='wall_avoidance',
            name='wall_avoidance_node',
            output='screen'
        ),

        #  9. Orange Box Detection Node (from vision_nodes)
        Node(
            package='vision_nodes',
            executable='orange_detector',
            name='orange_detector_node',
            output='screen'
        )
    ])
