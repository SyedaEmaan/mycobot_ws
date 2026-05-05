"""
csv_moveit.py — MoveIt + real EtherCAT hardware (joint 1 via mycobot_csv,
joints 2-6 + gripper via mock_components/GenericSystem).

Brings up:
  - ros2_control_node hosting the hybrid hardware (mycobot_csv + mock_components)
  - robot_state_publisher (wallclock; not sim)
  - joint_state_broadcaster (after 4s — lets EtherCAT bring-up finish)
  - arm_controller (JointTrajectoryController, after JSB spawner exits)
  - move_group + RViz (after 10s)

PRE-REQ: ros2_control_node must have cap_net_raw set:
    sudo setcap cap_net_raw,cap_net_admin=eip \\
      $(readlink -f $(ros2 pkg prefix controller_manager)/lib/controller_manager/ros2_control_node)
"""
import os
import yaml
from launch import LaunchDescription
from launch.actions import RegisterEventHandler, SetEnvironmentVariable, TimerAction
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except Exception:
        return None


def generate_launch_description():
    pkg_moveit = get_package_share_directory('mycobot_moveit_config')

    urdf_path = os.path.join(pkg_moveit, 'config', 'mycobot_280.urdf.xacro')

    robot_description_content = Command(
        [PathJoinSubstitution([FindExecutable(name="xacro")]), " ", urdf_path]
    )
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    moveit_config = (
        MoveItConfigsBuilder("mycobot_280", package_name="mycobot_moveit_config")
        .robot_description(file_path=urdf_path)
        .to_moveit_configs()
    )

    ompl_planning_yaml = load_yaml("mycobot_moveit_config", "config/ompl_planning.yaml")
    controllers_yaml = os.path.join(pkg_moveit, 'config', 'ros2_controllers.yaml')

    # 1. ros2_control_node — owns the hardware (mycobot_csv + mock_components)
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, controllers_yaml],
        remappings=[("~/robot_description", "/robot_description")],
        output="screen",
    )

    # 2. Robot state publisher (wallclock)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description],
    )

    # 3. Controller spawners
    #    JSB:  TimerAction-gated (lets EtherCAT bring-up finish before claim).
    #    Arm:  chained on JSB spawner exit, so we don't race JSB load.
    jsb_spawner_node = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )

    arm_spawner_node = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller"],
        output="screen",
    )

    spawn_jsb_delayed = TimerAction(
        period=4.0,
        actions=[jsb_spawner_node],
    )

    spawn_arm_after_jsb = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=jsb_spawner_node,
            on_exit=[arm_spawner_node],
        )
    )

    # 4. Move Group
    move_group_params = moveit_config.to_dict()
    if ompl_planning_yaml:
        move_group_params.update({"ompl": ompl_planning_yaml})

    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[move_group_params],
    )

    # 5. RViz
    rviz_config = moveit_config.to_dict()
    if ompl_planning_yaml:
        rviz_config.update({"ompl": ompl_planning_yaml})

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(pkg_moveit, 'config', 'moveit.rviz')],
        parameters=[rviz_config],
    )

    return LaunchDescription([
        # Pin RMW so ros2_control_node doesn't fail on missing cyclonedds.
        SetEnvironmentVariable("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp"),
        ros2_control_node,
        robot_state_publisher,
        spawn_jsb_delayed,
        spawn_arm_after_jsb,
        TimerAction(period=10.0, actions=[move_group_node, rviz_node]),
    ])
