#include <rclcpp/rclcpp.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include "trajectory_analysis/dynamics.hpp"
#include <unordered_map>

class TrajectoryListener : public rclcpp::Node
{
public:
  TrajectoryListener() : Node("trajectory_listener")
  {
    sub_ = this->create_subscription<moveit_msgs::msg::DisplayTrajectory>(
      "/display_planned_path",
      rclcpp::QoS(10),
      std::bind(&TrajectoryListener::callback, this, std::placeholders::_1)
    );
  }

private:
  // Full set of joints Pinocchio expects (order must match model.nq)
  const std::vector<std::string> full_joint_order = {
    "link1_to_link2",
    "link2_to_link3",
    "link3_to_link4",
    "link4_to_link5",
    "link5_to_link6",
    "link6_to_link6flange",

    "gripper_controller",
    "gripper_base_to_gripper_left2",
    "gripper_left3_to_gripper_left1",
    "gripper_base_to_gripper_right3",
    "gripper_base_to_gripper_right2",
    "gripper_right3_to_gripper_right1"
  };

  // MoveIt only publishes these joints
  const std::vector<std::string> moveit_joints = {
    "link1_to_link2",
    "link2_to_link3",
    "link3_to_link4",
    "link4_to_link5",
    "link5_to_link6",
    "link6_to_link6flange",
    "gripper_controller"
  };

  // Mimic multipliers (taken from URDF)
  std::unordered_map<std::string, double> mimic_multiplier = {
    {"gripper_base_to_gripper_left2",   1.0},
    {"gripper_left3_to_gripper_left1", -1.0},
    {"gripper_base_to_gripper_right3", -1.0},
    {"gripper_base_to_gripper_right2", -1.0},
    {"gripper_right3_to_gripper_right1", 1.0}
  };

  void callback(const moveit_msgs::msg::DisplayTrajectory::SharedPtr msg)
  {
    if (msg->trajectory.empty())
      return;

    const auto& traj = msg->trajectory[0].joint_trajectory;

    if (traj.joint_names.size() != moveit_joints.size())
    {
      RCLCPP_ERROR(this->get_logger(),
        "MoveIt returned %zu joints, expected %zu.",
        traj.joint_names.size(), moveit_joints.size());
      return;
    }

    // Map MoveIt joint names → index for reading positions
    std::unordered_map<std::string, int> index_map;
    for (size_t i = 0; i < traj.joint_names.size(); i++)
      index_map[traj.joint_names[i]] = i;

    for (const auto& point : traj.points)
    {
      // Full joint vectors Pinocchio needs
      Eigen::VectorXd q(full_joint_order.size());
      Eigen::VectorXd qd(full_joint_order.size());
      Eigen::VectorXd qdd(full_joint_order.size());

      q.setZero();
      qd.setZero();
      qdd.setZero();

      // First fill arm + gripper_controller from MoveIt
      for (size_t i = 0; i < moveit_joints.size(); i++)
      {
        const std::string& name = moveit_joints[i];
        int idx = index_map[name];

        q[i]   = point.positions[idx];
        qd[i]  = point.velocities[idx];
        qdd[i] = point.accelerations[idx];
      }

      // Now apply mimic joint rules using the gripper_controller position
      double gc_pos = q[6];    // index 6 corresponds to gripper_controller
      double gc_vel = qd[6];
      double gc_acc = qdd[6];

      for (size_t i = 7; i < full_joint_order.size(); i++)
      {
        const std::string& name = full_joint_order[i];
        double mult = mimic_multiplier[name];

        q[i]   = mult * gc_pos;
        qd[i]  = mult * gc_vel;
        qdd[i] = mult * gc_acc;
      }

      // 1. Total torque (static + dynamic)
      Eigen::VectorXd tau_total = compute_torques(q, qd, qdd);

      // 2. Static torque (gravity only)
      Eigen::VectorXd qd_zero = Eigen::VectorXd::Zero(q.size());
      Eigen::VectorXd qdd_zero = Eigen::VectorXd::Zero(q.size());
      Eigen::VectorXd tau_static = compute_torques(q, qd_zero, qdd_zero);

      // 3. Dynamic torque
      Eigen::VectorXd tau_dynamic = tau_total - tau_static;

      Eigen::IOFormat fmt(Eigen::StreamPrecision, Eigen::DontAlignCols);
      std::stringstream ss;
      ss << tau_dynamic.transpose().format(fmt);

      RCLCPP_INFO(this->get_logger(), "Dynamic Torques: %s", ss.str().c_str());
    }
  }

  rclcpp::Subscription<moveit_msgs::msg::DisplayTrajectory>::SharedPtr sub_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TrajectoryListener>());
  rclcpp::shutdown();
  return 0;
}

