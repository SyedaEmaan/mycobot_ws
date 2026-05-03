#pragma once
#include <Eigen/Dense>

Eigen::VectorXd compute_torques(
    const Eigen::VectorXd& q,
    const Eigen::VectorXd& qd,
    const Eigen::VectorXd& qdd);
