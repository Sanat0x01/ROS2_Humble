from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        
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
