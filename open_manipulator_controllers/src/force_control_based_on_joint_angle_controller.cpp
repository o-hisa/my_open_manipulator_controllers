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
  本コントローラは、ジョイント角度を基準にした力制御を行う。
  1.
  　始点と終点の位置を指定し、その位置に移動するための軌道を生成する。
  2.
  　経過時間に対して、軌道上の位置、速度、加速度を計算する。
  3.
  　逆運動学を用いて、軌道上の位置、速度、加速度に対応するジョイント角度、速度、加速度を計算する。
  4.
 　 ジョイント角度、速度、加速度に対応するトルクを計算し、ジョイントに出力する。

  制御周期はできる限り短くすることが望ましい。
  ubuntu上でusbのレイテンシーを変更するために以下のコマンドを入力ことで1msecに設定する。
  　sudo chmod a+w /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
  　echo 1 > /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
  上記コマンドはPC起動毎に実行する必要がある。

  現状ではマニピュレータに加わる外力を推定できないため、目標位置の追従と、各軸のバネ・ダンバ特性のパラメータが共通になる。
  そのため、独立して特性を変更することは不可能である。
  （目標位置の追従を重視すると、バネ・ダンパの特性も強く、硬いマニピュレータ制御となる）
*/

#include "open_manipulator_controllers/force_control_based_on_joint_angle_controller.h"

namespace open_manipulator_controllers {
bool ForceControlBasedOnJointAngleController::init(
    hardware_interface::RobotHW* robot_hardware, ros::NodeHandle& node_handle) {
  // Joint interface
  effort_joint_interface_ =
      robot_hardware->get<hardware_interface::EffortJointInterface>();
  if (effort_joint_interface_ == nullptr) {
    ROS_ERROR(
        "[ForceControlBasedOnJointAngleController] Could not get effort joint "
        "interface "
        "from hardware!");
    return false;
  }

  // Joint handle
  if (!node_handle.getParam("joint_names", joint_names_)) {
    ROS_ERROR(
        "[ForceControlBasedOnJointAngleController] Could not parse joint "
        "names");
    return false;
  }
  for (size_t i = 0; i < joint_names_.size(); i++) {
    try {
      effort_joint_handles_.push_back(
          effort_joint_interface_->getHandle(joint_names_[i]));
    } catch (const hardware_interface::HardwareInterfaceException& e) {
      ROS_ERROR_STREAM(
          "[ForceControlBasedOnJointAngleController] Could not get joint "
          "handle: "
          << e.what());
      return false;
    }
  }

  // KDL
  urdf::Model urdf;
  if (!urdf.initParam("robot_description")) {
    ROS_ERROR(
        "[ForceControlBasedOnJointAngleController] Could not parse urdf file");
    return false;
  }
  if (!kdl_parser::treeFromUrdfModel(urdf, kdl_tree_)) {
    ROS_ERROR(
        "[ForceControlBasedOnJointAngleController] Could not construct kdl "
        "tree");
    return false;
  }
  if (!node_handle.getParam("root_link", root_name_)) {
    ROS_ERROR(
        "[ForceControlBasedOnJointAngleController] Could not find root link "
        "name");
    return false;
  }
  if (!node_handle.getParam("tip_link", tip_name_)) {
    ROS_ERROR(
        "[ForceControlBasedOnJointAngleController] Could not find tip link "
        "name");
    return false;
  }
  if (!kdl_tree_.getChain(root_name_, tip_name_, kdl_chain_)) {
    ROS_ERROR(
        "[ForceControlBasedOnJointAngleController] Could not get KDL chain "
        "from tree");
    return false;
  }

  // Resize the variables
  q_.resize(kdl_chain_.getNrOfJoints());
  q_dot_.resize(kdl_chain_.getNrOfJoints());
  planning_q_.resize(kdl_chain_.getNrOfJoints());
  planning_q_dot_.resize(kdl_chain_.getNrOfJoints());
  planning_q_dot_dot_.resize(kdl_chain_.getNrOfJoints());
  tau_.resize(kdl_chain_.getNrOfJoints());
  G_.resize(kdl_chain_.getNrOfJoints());
  C_.resize(kdl_chain_.getNrOfJoints());
  H_.resize(kdl_chain_.getNrOfJoints());
  // p_out.resize(kdl_chain_.getNrOfJoints());

  // Gravity torque
  KDL::Vector g(0.0, 0.0, -9.81);
  grav_ = g;
  MCG_solver_.reset(new KDL::ChainDynParam(kdl_chain_, grav_));
  FK_solver_.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain_));
  J_solver_.reset(new KDL::ChainJntToJacSolver(kdl_chain_));
  J_dot_solver_.reset(new KDL::ChainJntToJacDotSolver(kdl_chain_));
  // IK_solver_acc_.reset(new KDL::ChainIkSolverAcc());
  IK_solver_vel_.reset(new KDL::ChainIkSolverVel_pinv(kdl_chain_));
  IK_solver_pos_.reset(new KDL::ChainIkSolverPos_LMA(kdl_chain_));

  // KDL::Frame start_pose(KDL::Rotation::RPY(0, 0, 0),
  //                       KDL::Vector(0.10, 0.1, 0.20));
  // KDL::Frame end_pose(KDL::Rotation::RPY(0, 0, 0),
  //                     KDL::Vector(0.10, -0.1, 0.20));
  KDL::Frame start_pose(KDL::Rotation::RPY(0, 0, 0),
                        KDL::Vector(0.25, 0.0, 0.05));
  KDL::Frame end_pose(KDL::Rotation::RPY(0, 0, 0),
                      KDL::Vector(0.15, 0.0, 0.25));
  path_ = new KDL::Path_Line(
      start_pose, end_pose, new KDL::RotationalInterpolation_SingleAxis(), 0.5);
  velpref_ = new KDL::VelocityProfile_Trap(0.01, 0.01);
  velpref_->SetProfile(0, path_->PathLength());
  traj_ = new KDL::Trajectory_Segment(path_, velpref_);
  ctraj_ = new KDL::Trajectory_Composite();
  ctraj_->Add(traj_);
  ctraj_->Add(new KDL::Trajectory_Stationary(
      1.0, KDL::Frame(KDL::Rotation::RPY(0.0, 0.0, 0.0),
                      KDL::Vector(0.15, 0.0, 0.25))));

  moving_start_time_ = ros::Time::now().toSec();
  return true;
}

void ForceControlBasedOnJointAngleController::planning_trajectory(
    double current_time) {
  double spend_time = current_time - moving_start_time_;

  current_pose_ = ctraj_->Pos(spend_time);
  current_vel_ = ctraj_->Vel(spend_time);
  current_acc_ = ctraj_->Acc(spend_time);
}

void ForceControlBasedOnJointAngleController::starting(const ros::Time& time) {}

void ForceControlBasedOnJointAngleController::update(
    const ros::Time& time, const ros::Duration& period) {
  // printf("time: %f\n", time.toSec());
  // printf("period: %f\n", period.toSec());

  // Get the current joint positions
  for (size_t i = 0; i < kdl_chain_.getNrOfJoints(); i++) {
    q_(i) = effort_joint_handles_[i].getPosition();
    q_dot_(i) = effort_joint_handles_[i].getVelocity();
  }

  MCG_solver_->JntToGravity(q_, G_);
  MCG_solver_->JntToCoriolis(q_, q_dot_, C_);
  MCG_solver_->JntToMass(q_, H_);

  // get gripper_position
  planning_trajectory(ros::Time::now().toSec());

  // solve IK from gripper_position
  IK_solver_pos_->CartToJnt(q_, current_pose_, planning_q_);
  IK_solver_vel_->CartToJnt(q_, current_vel_, planning_q_dot_);

  KDL::JntArrayVel q_dot_array(q_, q_dot_);
  KDL::Jacobian jac_dot_q(4);
  KDL::Jacobian jac_q(4);
  J_solver_->JntToJac(q_, jac_q);
  J_dot_solver_->JntToJacDot(q_dot_array, jac_dot_q);

  Eigen::Matrix<double, 6, 4> J;
  for (int i = 0; i < jac_q.rows(); i++) {
    for (int j = 0; j < jac_q.columns(); j++) {
      J(i, j) = jac_q(i, j);
    }
  };

  Eigen::JacobiSVD<Eigen::MatrixXd> svd(
      J, Eigen::ComputeFullU | Eigen::ComputeFullV);

  Eigen::VectorXd s = svd.singularValues();
  s = s.array().inverse();
  // 一般化逆行列の変数名は Ainv
  Eigen::Matrix<double, 4, 6> Jinv =
      svd.matrixV() * s.asDiagonal() * svd.matrixU().transpose();

  // const double kp = 250;
  // 制御周期1msec
  const double kp = 2000;
  const double kv = 20;

  // 制御周期16msec, 適切なパラメータが見つからなかった
  // const double kp = 1000;  // 2000;
  // const double kv = 2.0;   // 20;

  KDL::JntArray dest;
  dest.resize(kdl_chain_.getNrOfJoints());
  KDL::Twist tmp;
  KDL::MultiplyJacobian(jac_dot_q, planning_q_dot_, tmp);
  tmp = current_acc_ - tmp;

  Eigen::Matrix<double, 6, 1> dist;
  dist(0) = tmp.vel.x();
  dist(1) = tmp.vel.y();
  dist(2) = tmp.vel.z();
  dist(3) = tmp.rot.x();
  dist(4) = tmp.rot.y();
  dist(5) = tmp.rot.z();

  Eigen::MatrixXd tmp2;
  tmp2 = Jinv * dist;
  for (size_t i = 0; i < kdl_chain_.getNrOfJoints(); i++) {
    for (int i = 0; i < tmp2.rows(); i++) {
      planning_q_dot_dot_(i) = tmp2(i);
    }

    planning_q_dot_dot_(i) +=
        kp * (planning_q_(i) - q_(i)) + kv * (planning_q_dot_(i) - q_dot_(i));

    dest(i) = 0;
    for (int j = 0; j < H_.columns(); j++) {
      dest(i) += H_(i, j) * planning_q_dot_dot_(j);
    }
    tau_(i) = 0;
    tau_(i) += dest(i);
    tau_(i) += C_(i);
    tau_(i) += G_(i);
  }

  // Set effort command
  for (size_t i = 0; i < kdl_chain_.getNrOfJoints(); i++) {
    effort_joint_handles_[i].setCommand(tau_(i));
  }
}  // namespace open_manipulator_controllers

void ForceControlBasedOnJointAngleController::stopping(const ros::Time& time) {}

}  // namespace open_manipulator_controllers

PLUGINLIB_EXPORT_CLASS(
    open_manipulator_controllers::ForceControlBasedOnJointAngleController,
    controller_interface::ControllerBase)
