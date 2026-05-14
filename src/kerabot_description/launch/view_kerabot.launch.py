from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('kerabot_description')

    default_model = PathJoinSubstitution([pkg_share, 'urdf', 'kerabot.urdf.xacro'])
    default_rviz = PathJoinSubstitution([pkg_share, 'rviz', 'kerabot.rviz'])

    model_arg = DeclareLaunchArgument('model', default_value=default_model)
    rviz_arg = DeclareLaunchArgument('rvizconfig', default_value=default_rviz)

    robot_description = {
        'robot_description': ParameterValue(
            Command(['xacro ', LaunchConfiguration('model')]),
            value_type=str,
        ),
    }

    return LaunchDescription([
        model_arg,
        rviz_arg,
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[robot_description],
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', LaunchConfiguration('rvizconfig')],
        ),
    ])
