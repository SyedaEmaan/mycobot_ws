import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg_share = get_package_share_directory('mycobot_gazebo')
    
    # Use the simpler URDF without the problematic classic_gazebo include
    urdf_file = os.path.join(pkg_share, 'urdf', 'mycobot_280_classic_gazebo.urdf.xacro')
    
    robot_description = ParameterValue(Command(['xacro ', urdf_file]), value_type=str)
    
    # Controller config
    controller_config = os.path.join(pkg_share, 'config', 'mycobot_280_controllers.yaml')
    
    # Gazebo
    gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so'],
        output='screen')
    
    # Robot State Publisher
    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}])
    
    # Spawn robot
    spawn = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'mycobot', '-topic', 'robot_description', '-z', '0.5'],
        output='screen')
    
    # Load controllers after spawn
    load_joint_state_broadcaster = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn,
            on_exit=[
                Node(
                    package='controller_manager',
                    executable='spawner',
                    arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
                    parameters=[{'use_sim_time': True}])
            ]))
    
    load_arm_controller = TimerAction(
        period=3.0,
        actions=[Node(
            package='controller_manager',
            executable='spawner',
            arguments=['arm_controller', '--controller-manager', '/controller_manager'],
            parameters=[{'use_sim_time': True}])])
    
    return LaunchDescription([
        gazebo,
        robot_state_pub,
        spawn,
        load_joint_state_broadcaster,
        load_arm_controller
    ])
