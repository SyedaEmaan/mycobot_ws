import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    pkg_share = FindPackageShare(
        package='mycobot_gazebo'
    ).find('mycobot_gazebo')

    urdf_path = os.path.join(
        pkg_share,
        'urdf/ros2_control/classic_gazebo/mycobot_280_with_torque.urdf.xacro'
    )

    # ✅ Proper xacro execution (space between 'xacro' and file path)
    robot_description = ParameterValue(
        Command(['xacro', ' ', urdf_path]),
        value_type=str
    )

    # Start Gazebo (empty world)
    gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so'],
        output='screen'
    )

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            {
                'robot_description': robot_description,
                'use_sim_time': True
            }
        ],
        output='screen'
    )

    # Spawn robot into Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'mycobot_280',
            '-topic', 'robot_description',
            '-z', '0.5'
        ],
        output='screen'
    )

    # Load joint_state_broadcaster AFTER spawn
    load_joint_state_broadcaster = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_entity,
            on_exit=[
                Node(
                    package='controller_manager',
                    executable='spawner',
                    arguments=[
                        'joint_state_broadcaster',
                        '--controller-manager',
                        '/controller_manager'
                    ],
                    output='screen'
                )
            ]
        )
    )

    # Load arm_controller after short delay
    load_arm_controller = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=[
                    'arm_controller',
                    '--controller-manager',
                    '/controller_manager'
                ],
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_entity,
        load_joint_state_broadcaster,
        load_arm_controller
    ])
