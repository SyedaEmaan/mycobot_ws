#include "trajectory_analysis/dynamics.hpp"
#include <pinocchio/fwd.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

static pinocchio::Model model;
static pinocchio::Data data;
static bool loaded = false;

std::string package_share_directory =
    ament_index_cpp::get_package_share_directory("mycobot_gazebo");

std::string urdf_path = package_share_directory + "/urdf/ros2_control/gazebo/mycobot_280.urdf";

Eigen::VectorXd compute_torques(
    const Eigen::VectorXd& q,
    const Eigen::VectorXd& qd,
    const Eigen::VectorXd& qdd)
{
  if (!loaded)
  {
    pinocchio::urdf::buildModel(urdf_path, model);
    data = pinocchio::Data(model);
    loaded = true;
  }

  return pinocchio::rnea(model, data, q, qd, qdd);
}
