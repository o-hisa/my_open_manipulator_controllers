/*******************************************************************************
 * Copyright 2020 ROBOTIS CO., LTD.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *******************************************************************************/
/* Authors: hisayuki ogawa */
/*
  本コントローラは、手先位置を基準にした力制御を行う。
  1.
  　始点と終点の位置を指定し、その位置に移動するための軌道を生成する。
  2.
  　経過時間に対して、軌道上の位置、速度、加速度を計算する。
  3.
  　逆運動学を用いて、軌道上の位置、速度、加速度に対応するジョイント角度、速度を計算する。
  4.
　　ジョイントの角度、角速度から、手先の位置、速度に変換する
  5.
 　 手先位置を実現するために必要なトルクを計算し、ジョイントに出力する。

  制御周期はできる限り短くすることが望ましい。
  ubuntu上でusbのレイテンシーを変更するために以下のコマンドを入力ことで1msecに設定する。
  　sudo chmod a+w /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
  　echo 1 > /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
  上記コマンドはPC起動毎に実行する必要がある。

  現状ではマニピュレータに加わる外力を推定できないため、目標位置の追従と、各軸のバネ・ダンバ特性のパラメータが共通になる。
  そのため、独立して特性を変更することは不可能である。
  （目標位置の追従を重視すると、バネ・ダンパの特性も強く、硬いマニピュレータ制御となる）
*/

#include "open_manipulator_controllers/force_control_based_on_tip_position_controller.h"

namespace open_manipulator_controllers {
bool ForceControlBasedOnTipPositionController::init(
    hardware_interface::RobotHW* robot_hardware, ros::NodeHandle& node_handle) {
  // Joint interface
  effort_joint_interface_ =
      robot_hardware->get<hardware_interface::EffortJointInterface>();
  if (effort_joint_interface_ == nullptr) {
    ROS_ERROR(
        "[ForceControlBasedOnTipPositionController] Could not get effort joint "
        "interface "
        "from hardware!");
    return false;
  }

  // Joint handle
  if (!node_handle.getParam("joint_names", joint_names_)) {
    ROS_ERROR(
        "[ForceControlBasedOnTipPositionController] Could not parse joint "
        "names");
    return false;
  }
  for (size_t i = 0; i < joint_names_.size(); i++) {
    try {
      effort_joint_handles_.push_back(
          effort_joint_interface_->getHandle(joint_names_[i]));
    } catch (const hardware_interface::HardwareInterfaceException& e) {
      ROS_ERROR_STREAM(
          "[ForceControlBasedOnTipPositionController] Could not get joint "
          "handle: "
          << e.what());
      return false;
    }
  }

  // KDL
  urdf::Model urdf;
  if (!urdf.initParam("robot_description")) {
    ROS_ERROR(
        "[ForceControlBasedOnTipPositionController] Could not parse urdf file");
    return false;
  }
  if (!kdl_parser::treeFromUrdfModel(urdf, kdl_tree_)) {
    ROS_ERROR(
        "[ForceControlBasedOnTipPositionController] Could not construct kdl "
        "tree");
    return false;
  }
  if (!node_handle.getParam("root_link", root_name_)) {
    ROS_ERROR(
        "[ForceControlBasedOnTipPositionController] Could not find root link "
        "name");
    return false;
  }
  if (!node_handle.getParam("tip_link", tip_name_)) {
    ROS_ERROR(
        "[ForceControlBasedOnTipPositionController] Could not find tip link "
        "name");
    return false;
  }
  if (!kdl_tree_.getChain(root_name_, tip_name_, kdl_chain_)) {
    ROS_ERROR(
        "[ForceControlBasedOnTipPositionController] Could not get KDL chain "
        "from tree");
    return false;
  }

  // Resize the variables
  q_.resize(kdl_chain_.getNrOfJoints());
  q_dot_.resize(kdl_chain_.getNrOfJoints());
  tau_.resize(kdl_chain_.getNrOfJoints());
  G_.resize(kdl_chain_.getNrOfJoints());
  C_.resize(kdl_chain_.getNrOfJoints());
  H_.resize(kdl_chain_.getNrOfJoints());
  // p_out.resize(kdl_chain_.getNrOfJoints());

  // Gravity torque
  KDL::Vector g(0.0, 0.0, -9.81);
  grav_ = g;
  MCG_solver_.reset(new KDL::ChainDynParam(kdl_chain_, grav_));
  FK_solver_pos_.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain_));
  FK_solver_vel_.reset(new KDL::ChainFkSolverVel_recursive(kdl_chain_));
  J_solver_.reset(new KDL::ChainJntToJacSolver(kdl_chain_));
  J_dot_solver_.reset(new KDL::ChainJntToJacDotSolver(kdl_chain_));
  IK_solver_vel_.reset(new KDL::ChainIkSolverVel_pinv(kdl_chain_));
  IK_solver_pos_.reset(new KDL::ChainIkSolverPos_LMA(kdl_chain_));

  // KDL::Frame start_pose(KDL::Rotation::RPY(0, 0, 0),
  //                       KDL::Vector(0.00, 0.0, 0.25));
  // KDL::Frame end_pose(KDL::Rotation::RPY(0, 0, 0),
  //                     KDL::Vector(0.20, 0.0, 0.25));

  KDL::Frame start_pose(KDL::Rotation::RPY(0, 0, 0),
                        KDL::Vector(0.167, 0.0, 0.05));
  KDL::Frame end_pose(KDL::Rotation::RPY(0, 0, 0),
                      KDL::Vector(0.100, 0.0, 0.25));

  path_ = new KDL::Path_Line(
      start_pose, end_pose, new KDL::RotationalInterpolation_SingleAxis(), 0.5);
  velpref_ = new KDL::VelocityProfile_Trap(0.01, 0.01);
  velpref_->SetProfile(0, path_->PathLength());
  traj_ = new KDL::Trajectory_Segment(path_, velpref_);
  ctraj_ = new KDL::Trajectory_Composite();
  ctraj_->Add(traj_);
  ctraj_->Add(new KDL::Trajectory_Stationary(
      1.0, KDL::Frame(KDL::Rotation::RPY(0.0, 0.0, 0.0),
                      KDL::Vector(0.100, 0.0, 0.25))));

  moving_start_time_ = ros::Time::now().toSec();
  return true;
}

void ForceControlBasedOnTipPositionController::planning_trajectory(
    double current_time) {
  double spend_time = current_time - moving_start_time_;

  target_pose_ = ctraj_->Pos(spend_time);
  target_vel_ = ctraj_->Vel(spend_time);
  target_acc_ = ctraj_->Acc(spend_time);
}

void ForceControlBasedOnTipPositionController::transformKDLToEigen(
    const KDL::Frame& k, Eigen::Affine3d& e) {
  // KDL::Vector v;
  // KDL::SetToZero(v);
  // KDL::Rotation r(v, v, v);
  // KDL::Frame f(r, v);
  for (int i = 0; i < 3; i++) {
    // translation
    e(i, 3) = k.p(i);
    for (int j = 0; j < 3; j++) {
      // rotation
      e(i, j) = k.M(i, j);
    }
  }
  e(3, 0) = e(3, 1) = e(3, 2) = 0;
  e(3, 3) = 1;
}

void ForceControlBasedOnTipPositionController::twistKDLToEigen(
    const KDL::Twist& k, Eigen::Matrix<double, 6, 1>& e) {
  for (int i = 0; i < 6; i++) {
    e[i] = k[i];
  }
}

void ForceControlBasedOnTipPositionController::starting(const ros::Time& time) {
}

void ForceControlBasedOnTipPositionController::update(
    const ros::Time& time, const ros::Duration& period) {
  // printf("time: %f\n", time.toSec());
  // printf("period: %f\n", period.toSec());

  // Get the current joint positions
  for (size_t i = 0; i < kdl_chain_.getNrOfJoints(); i++) {
    q_(i) = effort_joint_handles_[i].getPosition();
    q_dot_(i) = effort_joint_handles_[i].getVelocity();
  }
  // transform KDL::JntArray to Eigen::Matrix
  Eigen::Matrix<double, 4, 1> q;
  Eigen::Matrix<double, 4, 1> q_dot;
  for (int i = 0; i < q_.rows(); i++) {
    for (int j = 0; j < q_.columns(); j++) {
      q(i, j) = q_(i);
      q_dot(i, j) = q_dot_(i);
    }
  }

  // Calculate M(q), C(q, q_dot), G(q)(row:6, column:1)
  MCG_solver_->JntToGravity(q_, G_);
  MCG_solver_->JntToCoriolis(q_, q_dot_, C_);
  MCG_solver_->JntToMass(q_, H_);

  // transform KDL::JntArray to Eigen::Matrix
  Eigen::Matrix<double, 4, 1> G;
  for (int i = 0; i < G_.rows(); i++) {
    for (int j = 0; j < G_.columns(); j++) {
      G(i, j) = G_(i);
    }
  }
  Eigen::Matrix<double, 4, 1> C;
  for (int i = 0; i < C_.rows(); i++) {
    for (int j = 0; j < C_.columns(); j++) {
      C(i, j) = C_(i);
    }
  }
  Eigen::Matrix<double, 4, 4> H;
  for (int i = 0; i < H_.rows(); i++) {
    for (int j = 0; j < H_.columns(); j++) {
      H(i, j) = H_(i, j);
    }
  }

  // get gripper_position on the time
  // update target_pose_, target_vel_, target_acc_
  //
  planning_trajectory(time.toSec());
  // transform KDL::Frame to Eigen::Matrix
  Eigen::Matrix<double, 6, 1> r_dist, r_dist_dot;
  twistKDLToEigen(target_vel_, r_dist_dot);
  r_dist(0) = target_pose_.p.x();
  r_dist(1) = target_pose_.p.y();
  r_dist(2) = target_pose_.p.z();
  target_pose_.M.GetEulerZYX(r_dist(3), r_dist(4), r_dist(5));

  // solve FK from gripper_position
  // get current position
  FK_solver_pos_->JntToCart(q_, current_pose_);
  KDL::JntArrayVel q_dot_array(q_, q_dot_);
  KDL::FrameVel current_vel_tmp;
  FK_solver_vel_->JntToCart(q_dot_array, current_vel_tmp);
  current_vel_ = current_vel_tmp.GetTwist();

  Eigen::Matrix<double, 6, 1> r, r_dot;
  twistKDLToEigen(current_vel_, r_dot);
  r(0) = current_pose_.p.x();
  r(1) = current_pose_.p.y();
  r(2) = current_pose_.p.z();
  current_pose_.M.GetEulerZYX(r(3), r(4), r(5));
  // printf("r:\n");
  // printf("%lf\n", r(0));
  // printf("%lf\n", r(1));
  // printf("%lf\n", r(2));

  KDL::Jacobian jac_dot_q(4);
  KDL::Jacobian jac_q(4);
  // calculate Jacobian(row:6, column:4)
  J_solver_->JntToJac(q_, jac_q);
  // calculate Jacobian dot(row:6, column:4)
  J_dot_solver_->JntToJacDot(q_dot_array, jac_dot_q);

  Eigen::Matrix<double, 6, 4> J, J_dot;
  for (int i = 0; i < 6; i++) {
    for (int j = 0; j < 4; j++) {
      J(i, j) = jac_q(i, j);
      J_dot(i, j) = jac_dot_q(i, j);
    }
  };

  Eigen::JacobiSVD<Eigen::MatrixXd> svd(
      J, Eigen::ComputeFullU | Eigen::ComputeFullV);

  Eigen::VectorXd s = svd.singularValues();
  s = s.array().inverse();
  // calculate Jinv(row:4, column:6)
  Eigen::Matrix<double, 4, 6> Jinv =
      svd.matrixV() * s.asDiagonal() * svd.matrixU().transpose();

  // calculate Jtrans(row:4, column:6)
  Eigen::Matrix<double, 4, 6> Jtrans = J.transpose();

  // setting parameters
  Eigen::Matrix<double, 6, 1> K_dist, D_dist;
  // 制御周期が16msec
  // パラメータによる暴走(目標位置の発散)が発生しやすい
  K_dist(0, 0) = 200.0;
  K_dist(1, 0) = 200.0;
  K_dist(2, 0) = 200.0;
  K_dist(3, 0) = 0.0;
  K_dist(4, 0) = 0.0;
  K_dist(5, 0) = 0.0;
  D_dist(0, 0) = 1;
  D_dist(1, 0) = 1;
  D_dist(2, 0) = 1;
  D_dist(3, 0) = 0;
  D_dist(4, 0) = 0;
  D_dist(5, 0) = 0;

  // // 制御周期が1msec, 強め
  // K_dist(0, 0) = 700.0;
  // K_dist(1, 0) = 700.0;
  // K_dist(2, 0) = 700.0;
  // K_dist(3, 0) = 5.0;
  // K_dist(4, 0) = 5.0;
  // K_dist(5, 0) = 5.0;
  // D_dist(0, 0) = 10;
  // D_dist(1, 0) = 10;
  // D_dist(2, 0) = 10;
  // D_dist(3, 0) = 0;
  // D_dist(4, 0) = 0;
  // D_dist(5, 0) = 0;

  // // 制御周期が1msec, 弱め
  // K_dist(0, 0) = 200.0;
  // K_dist(1, 0) = 200.0;
  // K_dist(2, 0) = 200.0;
  // K_dist(3, 0) = 0.0;
  // K_dist(4, 0) = 0.0;
  // K_dist(5, 0) = 0.0;
  // D_dist(0, 0) = 2;
  // D_dist(1, 0) = 2;
  // D_dist(2, 0) = 2;
  // D_dist(3, 0) = 0;
  // D_dist(4, 0) = 0;
  // D_dist(5, 0) = 0;

  // calculate tau(row:4, column:1)
  Eigen::Matrix<double, 4, 1> tau1 = -H * Jinv * J_dot * q_dot + C + G;

  Eigen::Matrix<double, 4, 1> tau2 =
      -Jtrans * (D_dist.cwiseProduct(r_dot - r_dist_dot) +
                 K_dist.cwiseProduct(r - r_dist));
  Eigen::Matrix<double, 4, 1> tau = tau1 + tau2;

  // printf("tau1:\n");
  // for (int i = 0; i < 4; i++) {
  //   printf("%lf\n", tau1(i));
  // }
  // printf("\n");
  // printf("tau2:\n");
  // for (int i = 0; i < 4; i++) {
  //   printf("%lf\n", tau2(i));
  // }
  // printf("\n");

  // printf("r:\n");
  // for (int i = 0; i < 6; i++) {
  //   printf("%lf\n", r(i));
  // }
  // printf("\n");
  // printf("r_dist:\n");
  // for (int i = 0; i < 6; i++) {
  //   printf("%lf\n", r_dist(i));
  // }
  // printf("\n");

  // Set effort command
  // printf("tau:\n");
  for (size_t i = 0; i < kdl_chain_.getNrOfJoints(); i++) {
    // printf("%lf\n", tau(i));
    effort_joint_handles_[i].setCommand(tau(i));
  }
  // printf("\n");

}  // namespace open_manipulator_controllers

void ForceControlBasedOnTipPositionController::stopping(const ros::Time& time) {
}

}  // namespace open_manipulator_controllers

PLUGINLIB_EXPORT_CLASS(
    open_manipulator_controllers::ForceControlBasedOnTipPositionController,
    controller_interface::ControllerBase)
