from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_patrol',
            executable='patrol_with_service',
            name='patrol_with_service'
        ),
        Node(
            package='robot_patrol',
            executable='direction_service',
            output='screen')
    ])
