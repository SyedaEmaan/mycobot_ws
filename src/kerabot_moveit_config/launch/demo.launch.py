import os

# kerabot.ros2_control.xacro uses the mycobot_csv plugin (real EtherCAT, not mock),
# so demo.launch.py requires the EtherCAT bus + drives powered on. Pin RMW to
# fastrtps before the builder runs so ros2_control_node doesn't try (and fail) to
# dlopen cyclonedds, which is the env default but isn't installed.
os.environ["RMW_IMPLEMENTATION"] = "rmw_fastrtps_cpp"

from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder(
        "my_robot_arm", package_name="kerabot_moveit_config"
    ).to_moveit_configs()
    return generate_demo_launch(moveit_config)
