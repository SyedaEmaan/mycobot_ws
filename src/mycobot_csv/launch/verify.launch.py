"""
mycobot_csv verification launch.

Brings up:
  - ros2_control_node loading the MyCobotCSV plugin
  - robot_state_publisher (so /joint_states reflects the live encoder)
  - joint_state_broadcaster (after a 3s delay to let hardware activate)
  - forward_velocity_controller (after a 4s delay)

USAGE:
    ros2 launch mycobot_csv verify.launch.py
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction


def generate_launch_description():
    pkg_share = get_package_share_directory("mycobot_csv")
    urdf_file = os.path.join(pkg_share, "urdf", "test_one_joint.urdf")
    with open(urdf_file, "r") as f:
        robot_description = f.read()

    controllers_yaml = os.path.join(pkg_share, "config", "controllers.yaml")

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"robot_description": robot_description},
            controllers_yaml,
        ],
        remappings=[("~/robot_description", "/robot_description")],
        output="screen",
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
        output="screen",
    )

    joint_state_broadcaster_spawner = TimerAction(
        period=3.0,
        actions=[Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster",
                       "--controller-manager", "/controller_manager"],
            output="screen",
        )],
    )

    forward_velocity_controller_spawner = TimerAction(
        period=4.0,
        actions=[Node(
            package="controller_manager",
            executable="spawner",
            arguments=["forward_velocity_controller",
                       "--controller-manager", "/controller_manager"],
            output="screen",
        )],
    )

    return LaunchDescription([
        ros2_control_node,
        robot_state_publisher,
        joint_state_broadcaster_spawner,
        forward_velocity_controller_spawner,
    ])
