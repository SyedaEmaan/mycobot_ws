import os
import yaml
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, RegisterEventHandler
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
    except Exception as e:
        return None

def generate_launch_description():
    pkg_gazebo = get_package_share_directory('mycobot_gazebo')
    pkg_moveit = get_package_share_directory('mycobot_moveit_config')
    
    urdf_path = os.path.join(pkg_gazebo, 'urdf', 'ros2_control', 'classic_gazebo', 'mycobot_280_gazebo_classic.urdf.xacro')
    
    robot_description_content = Command(
        [PathJoinSubstitution([FindExecutable(name="xacro")]), " ", urdf_path]
    )
    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}
    
    moveit_config = (
        MoveItConfigsBuilder("mycobot_280", package_name="mycobot_moveit_config")
        .robot_description(file_path=urdf_path)
        .to_moveit_configs()
    )
    
    # Load OMPL planning config manually
    ompl_planning_yaml = load_yaml("mycobot_moveit_config", "config/ompl_planning.yaml")
    
    # 1. Start Gazebo
    gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so'],
        output='screen'
    )
    
    # 2. Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': True}]
    )
    
    # 3. Spawn Robot
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'mycobot_280', '-topic', 'robot_description', '-z', '0.2'],
        output='screen'
    )
    
    # 4. Controller Spawners
    spawn_jsb = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )
    spawn_arm_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller"],
        output="screen",
    )
    
    # 5. Move Group - WITH OMPL CONFIG
    move_group_params = moveit_config.to_dict()
    move_group_params.update({'use_sim_time': True})
    
    # Add OMPL planning config
    if ompl_planning_yaml:
        move_group_params.update({"ompl": ompl_planning_yaml})
    
    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[move_group_params],
    )
    
    # 6. RViz
    rviz_config = moveit_config.to_dict()
    rviz_config.update({'use_sim_time': True})
    
    if ompl_planning_yaml:
        rviz_config.update({"ompl": ompl_planning_yaml})
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(pkg_moveit, 'config', 'moveit.rviz')],
        parameters=[rviz_config],
    )
    
    # Event Handlers
    load_jsb = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[spawn_jsb],
        )
    )
    
    load_arm_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_jsb,
            on_exit=[spawn_arm_controller],
        )
    )
    
    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_entity,
        load_jsb,
        load_arm_controller,
        TimerAction(period=10.0, actions=[move_group_node, rviz_node]),
    ])
