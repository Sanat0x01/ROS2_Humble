from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_robot_controller',
            executable='test_node',
            name='test_node',
            output='screen'
        ),
        Node(
            package='my_robot_controller',
            executable='draw_circle',
            name='draw_circle',
            output='screen'
        ),
        Node(
            package='my_robot_controller',
            executable='pose_subscriber',
            name='pose_subscriber',
            output='screen'
        ),
        Node(
            package='my_robot_controller',
            executable='number_publisher',
            name='number_publisher',
            output='screen'
        ),
        Node(
            package='my_robot_controller',
            executable='number_subscriber',
            name='number_subscriber',
            output='screen'
        ),
    ])
