"""
verify_five.launch.py — five-joint CSV verification.

Same structure as verify_four.launch.py but uses test_five_joints.urdf and
controllers_five.yaml. forward_velocity_controller takes a Float64MultiArray
of five velocities (one per joint).
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import RegisterEventHandler, SetEnvironmentVariable, TimerAction
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("mycobot_csv")
    urdf_file = os.path.join(pkg_share, "urdf", "test_five_joints.urdf")
    with open(urdf_file, "r") as f:
        robot_description_content = f.read()

    controllers_yaml = os.path.join(pkg_share, "config", "controllers_five.yaml")

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description_content}],
        output="screen",
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{"robot_description": robot_description_content}, controllers_yaml],
        remappings=[("~/robot_description", "/robot_description")],
        output="screen",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    forward_velocity_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["forward_velocity_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    delayed_jsb = TimerAction(period=4.0, actions=[joint_state_broadcaster_spawner])

    velocity_after_jsb = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[forward_velocity_controller_spawner],
        )
    )

    return LaunchDescription([
        SetEnvironmentVariable("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp"),
        robot_state_publisher,
        ros2_control_node,
        delayed_jsb,
        velocity_after_jsb,
    ])
