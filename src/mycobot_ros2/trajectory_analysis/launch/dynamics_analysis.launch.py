from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="trajectory_analysis",
            executable="trajectory_listener",
            name="trajectory_listener"
        )
    ])
