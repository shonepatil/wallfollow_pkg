from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='wallfollow_pkg',
            executable='my_action_server',
            output='screen'),
        Node(
            package='wallfollow_pkg',
            executable='wallfollow',
            output='screen'),
    ])
