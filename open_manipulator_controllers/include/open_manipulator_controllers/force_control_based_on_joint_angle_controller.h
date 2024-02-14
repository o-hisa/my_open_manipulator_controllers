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

/* Authors: Ryan Shim */

#ifndef OPEN_MANIPULATOR_CONTROLLERS_FORCE_CONTROL_BASED_ON_JOINT_ANGLE_CONTROLLER_H
#define OPEN_MANIPULATOR_CONTROLLERS_FORCE_CONTROL_BASED_ON_JOINT_ANGLE_CONTROLLER_H

#include <boost/scoped_ptr.hpp>
#include <string>
#include <vector>

#include <assert.h>
#include <controller_interface/controller.h>
#include <controller_interface/controller_base.h>
#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/hardware_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <pluginlib/class_list_macros.h>
#include <ros/node_handle.h>
#include <ros/ros.h>
#include <ros/time.h>
#include <urdf/model.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/LU>
#include <kdl/chain.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainjnttojacdotsolver.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/frames.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/jntarrayvel.hpp>
#include <kdl/jntspaceinertiamatrix.hpp>
#include <kdl/kdl.hpp>
#include <kdl/path.hpp>
#include <kdl/path_line.hpp>
#include <kdl/path_roundedcomposite.hpp>
#include <kdl/rotational_interpolation_sa.hpp>
#include <kdl/trajectory.hpp>
#include <kdl/trajectory_composite.hpp>
#include <kdl/trajectory_segment.hpp>
#include <kdl/trajectory_stationary.hpp>
#include <kdl/tree.hpp>
#include <kdl/velocityprofile_trap.hpp>
#include <kdl_parser/kdl_parser.hpp>

namespace open_manipulator_controllers {
class ForceControlBasedOnJointAngleController
    : public controller_interface::MultiInterfaceController<
          hardware_interface::EffortJointInterface> {
 public:
  bool init(hardware_interface::RobotHW* robot_hardware,
            ros::NodeHandle& node_handle) override;
  void starting(const ros::Time& time) override;
  void update(const ros::Time& time, const ros::Duration& period) override;
  void stopping(const ros::Time& time) override;
  void planning_trajectory(double current_time);

 private:
  // Joint handle
  hardware_interface::EffortJointInterface* effort_joint_interface_;
  std::vector<hardware_interface::JointHandle> effort_joint_handles_;
  std::vector<std::string> joint_names_;
  std::string root_name_;
  std::string tip_name_;

  // KDL
  KDL::Tree kdl_tree_;
  KDL::Chain kdl_chain_;
  KDL::JntArray q_;
  KDL::JntArray q_dot_;
  KDL::JntArray planning_q_;
  KDL::JntArray planning_q_dot_;
  KDL::JntArray planning_q_dot_dot_;
  KDL::JntArray tau_;
  KDL::Vector grav_;
  KDL::JntArray G_;
  KDL::JntArray C_;
  KDL::JntSpaceInertiaMatrix H_;
  KDL::Path_Line* path_;
  KDL::Trajectory* traj_;
  KDL::Trajectory_Composite* ctraj_;
  KDL::Trajectory_Stationary* traj;
  KDL::VelocityProfile* velpref_;
  KDL::Frame current_pose_;
  KDL::Twist current_vel_;
  KDL::Twist current_acc_;
  double moving_start_time_;
  double gripper_position_[3] = {0, 0, 0};

  // KDL solver
  boost::scoped_ptr<KDL::ChainDynParam> MCG_solver_;
  boost::scoped_ptr<KDL::ChainJntToJacSolver> J_solver_;
  boost::scoped_ptr<KDL::ChainJntToJacDotSolver> J_dot_solver_;
  boost::scoped_ptr<KDL::ChainFkSolverPos_recursive> FK_solver_;
  boost::scoped_ptr<KDL::ChainIkSolverVel_pinv> IK_solver_vel_;
  boost::scoped_ptr<KDL::ChainIkSolverPos_LMA> IK_solver_pos_;
};
}  // namespace open_manipulator_controllers
#endif  // OPEN_MANIPULATOR_CONTROLLERS_FORCE_CONTROL_BASED_ON_JOINT_ANGLE_CONTROLLER_H
