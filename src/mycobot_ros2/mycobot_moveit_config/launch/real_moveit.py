import os
import xacro
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    pkg_moveit = get_package_share_directory('mycobot_moveit_config')

    # 1. Define Paths
    urdf_path = os.path.join(pkg_moveit, "config", "mycobot_280.urdf.xacro")
    ros2_controllers_path = os.path.join(pkg_moveit, "config", "ros2_controllers.yaml")

    # 2. Process URDF with Xacro (Direct Python Processing)
    # This ignores the stderr warnings that were killing your previous launch
    doc = xacro.process_file(urdf_path)
    robot_description = {"robot_description": doc.toxml()}

    # 3. Build MoveIt Config
    moveit_config = (
        MoveItConfigsBuilder("mycobot_280", package_name="mycobot_moveit_config")
        .robot_description(file_path=urdf_path)
        .to_moveit_configs()
    )

    # 4. Define Nodes
    
    # ✅ Controller Manager
    # We pass the 'robot_description' string we just generated
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, ros2_controllers_path],
        output="screen",
    )

    # ✅ Joint State Broadcaster
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )

    # ✅ Arm Controller
    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller"],
        output="screen",
    )

    # ✅ Move Group
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
    )

    # ✅ RViz
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=['-d', os.path.join(pkg_moveit, 'config', 'moveit.rviz')],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
        ],
    )

    return LaunchDescription([
        control_node,
        joint_state_broadcaster_spawner,
        arm_controller_spawner,
        move_group_node,
        rviz_node,
    ])
