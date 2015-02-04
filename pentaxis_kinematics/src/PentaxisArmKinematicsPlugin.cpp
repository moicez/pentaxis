/**
 * @file PentaxisArmKinematicsPlugin.cpp
 * @author Rasmus Hasle Andersen, Moises Estrada Casta/neda <moises@gmail.com>
 * @version 1.0
 *
 * @section LICENSE
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details at
 * http://www.gnu.org/copyleft/gpl.html
 *
 * @section DESCRIPTION
 *
 * This is the Moveit! Plugin which implements the base class kinematics::KinematicsBase
 */


#include <iostream>
#include <fstream>

#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <kdl/frames_io.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <eigen_conversions/eigen_msg.h>
#include <eigen_conversions/eigen_kdl.h>
#include <algorithm>
#include <numeric>

#include <pluginlib/class_list_macros.h>

#include <boost/current_function.hpp>

#include "PentaxisArmKinematicsPlugin.h"
#include "PentaxisIK/PentaxisArmAnalyticalIK.h"
#include "Kinematicshelper.h"


using namespace KDL;
using namespace std;

PLUGINLIB_EXPORT_CLASS(pentaxis_kinematics::PentaxisArmKinematicsPlugin, kinematics::KinematicsBase)

namespace pentaxis_kinematics {

PentaxisArmKinematicsPlugin::PentaxisArmKinematicsPlugin() :
    active_(false), dimension_(5), free_angle_(0)
{
}

bool
PentaxisArmKinematicsPlugin::isActive()
{
  if (active_)
    return true;
  return false;
}

void
PentaxisArmKinematicsPlugin::setRobotModel(
    boost::shared_ptr<urdf::ModelInterface>& robot_model)
{
  robot_model_ = robot_model;
}

bool PentaxisArmKinematicsPlugin::initialize(const std::string& robot_description,
                                           const std::string& group_name,
                                           const std::string& base_name,
                                           const std::string& tip_name,
                                           double search_discretization)
{
  setValues(robot_description, group_name, base_name, tip_name,search_discretization);

  std::string xml_string;
  dimension_ = 5;

  ros::NodeHandle node_handle("~/"+group_name);

  boost::shared_ptr<urdf::ModelInterface> robot_model_interface;
  urdf::Model *robot_model = new urdf::Model();

  std::string urdf_xml,full_urdf_xml;
  node_handle.param(robot_description,urdf_xml,std::string(""));

  if (!node_handle.getParam(robot_description, urdf_xml))
  {
    ROS_FATAL("Could not load the xml from parameter server: %s\n", urdf_xml.c_str());
    return false;
  }

  robot_model->initString(urdf_xml);

  robot_model_interface.reset(robot_model);

  setRobotModel(robot_model_interface);

  ROS_DEBUG("Loading KDL Tree");
  if(!kinematics_helper::getKDLChain(*robot_model_,base_frame_,tip_frame_,kdl_chain_))
  {
    active_ = false;
    ROS_ERROR("Could not load kdl tree");
  }

  ik_.reset(
      new pentaxis_kinematics::PentaxisArmAnalyticalIK(*robot_model_,
                                                   robot_description,
                                base_name,
                                tip_name));

  pentaxis_arm_ik_solver_.reset(
      new pentaxis_kinematics::PentaxisArmIKSolver(
          *ik_,
          *robot_model_.get(),
          base_frame_,
          tip_frame_,
          search_discretization_,
          free_angle_));

  if(!pentaxis_arm_ik_solver_->active_)
  {
    ROS_ERROR("Could not load ik");
    active_ = false;
  }
  else
  {
    pentaxis_arm_ik_solver_->getSolverInfo(ik_solver_info_);
    kinematics_helper::getKDLChainInfo(kdl_chain_,fk_solver_info_);
    fk_solver_info_.joint_names = ik_solver_info_.joint_names;

    for(unsigned int i=0; i < ik_solver_info_.joint_names.size(); i++)
    {
      ROS_DEBUG("PentaxisKinematics:: joint name: %s",ik_solver_info_.joint_names[i].c_str());
    }
    for(unsigned int i=0; i < ik_solver_info_.link_names.size(); i++)
    {
      ROS_DEBUG("PentaxisKinematics can solve IK for %s",ik_solver_info_.link_names[i].c_str());
    }
    for(unsigned int i=0; i < fk_solver_info_.link_names.size(); i++)
    {
      ROS_DEBUG("PentaxisKinematics can solve FK for %s",fk_solver_info_.link_names[i].c_str());
    }
    ROS_DEBUG("PentaxisKinematics::active for %s",group_name.c_str());
    active_ = true;
  }
  return active_;
}

bool
PentaxisArmKinematicsPlugin::getPositionIK(
    const geometry_msgs::Pose &ik_pose,
    const std::vector<double> &ik_seed_state,
    std::vector<double> &solution,
    moveit_msgs::MoveItErrorCodes &error_code,
    const kinematics::KinematicsQueryOptions &options) const
{
  ROS_ERROR("1 The function is not yet implemented: %s", BOOST_CURRENT_FUNCTION);
  return false;
}

bool PentaxisArmKinematicsPlugin::searchPositionIK(const geometry_msgs::Pose &ik_pose,
                                                 const std::vector<double> &ik_seed_state,
                                                 double timeout,
                                                 std::vector<double> &solution,
                                                 moveit_msgs::MoveItErrorCodes &error_code,
                                                 const kinematics::KinematicsQueryOptions &options) const
{
  if(!active_)
  {
    ROS_ERROR("kinematics not active");
    error_code.val = error_code.PLANNING_FAILED;
    return false;
  }
  KDL::Frame pose_desired;
  Eigen::Affine3d tp;
  tf::poseMsgToEigen(ik_pose, tp);
  tf::transformEigenToKDL(tp, pose_desired);

  //Do the IK
  KDL::JntArray jnt_pos_in;
  KDL::JntArray jnt_pos_out;
  jnt_pos_in.resize(dimension_);
  for(int i=0; i < dimension_; i++)
  {
    jnt_pos_in(i) = ik_seed_state[i];
  }

  int ik_valid = pentaxis_arm_ik_solver_->CartToJntSearch(jnt_pos_in,
                                                     pose_desired,
                                                     jnt_pos_out,
                                                     timeout);
  if(ik_valid == pentaxis_kinematics::NO_IK_SOLUTION)
  {
    error_code.val = error_code.NO_IK_SOLUTION;
    ROS_DEBUG("No valid inverse kinematics found..");
    return false;
  }

  if(ik_valid >= 0)
  {
    solution.resize(dimension_);
    for(int i=0; i < dimension_; i++)
    {
      solution[i] = jnt_pos_out(i);
    }
    error_code.val = error_code.SUCCESS;
    return true;
  }
  else
  {
    ROS_DEBUG("An IK solution could not be found");
    error_code.val = error_code.NO_IK_SOLUTION;
    return false;
  }
}

bool PentaxisArmKinematicsPlugin::searchPositionIK(const geometry_msgs::Pose &ik_pose,
                                                 const std::vector<double> &ik_seed_state,
                                                 double timeout,
                                                 const std::vector<double> &consistency_limit,
                                                 std::vector<double> &solution,
                                                 moveit_msgs::MoveItErrorCodes &error_code,
                                                 const kinematics::KinematicsQueryOptions &options) const
{
	//return searchPositionIK(ik_pose, ik_seed_state, timeout, solution, error_code, options);

  ROS_ERROR("2 The function is not yet implemented: %s", BOOST_CURRENT_FUNCTION);
  return false;
}

bool PentaxisArmKinematicsPlugin::searchPositionIK(const geometry_msgs::Pose &ik_pose,
                                                 const std::vector<double> &ik_seed_state,
                                                 double timeout,
                                                 std::vector<double> &solution,
                                                 const IKCallbackFn &solution_callback,
                                                 moveit_msgs::MoveItErrorCodes &error_code,
                                                 const kinematics::KinematicsQueryOptions &options) const
{
  ROS_ERROR("3 The function is not yet implemented: %s", BOOST_CURRENT_FUNCTION);
  return false;
}

bool
PentaxisArmKinematicsPlugin::searchPositionIK(
    const geometry_msgs::Pose &ik_pose,
    const std::vector<double> &ik_seed_state,
    double timeout,
    const std::vector<double> &consistency_limit,
    std::vector<double> &solution,
    const IKCallbackFn &solution_callback,
    moveit_msgs::MoveItErrorCodes &error_code,
    const kinematics::KinematicsQueryOptions &options) const
    {

  return searchPositionIK(ik_pose,
                          ik_seed_state,
                          timeout,
                          solution,
                          error_code,
                          options);

}

bool PentaxisArmKinematicsPlugin::getPositionFK(const std::vector<std::string> &link_names,
                                              const std::vector<double> &joint_angles,
                                              std::vector<geometry_msgs::Pose> &poses) const
{
  ROS_ERROR("Now in: %s", BOOST_CURRENT_FUNCTION);
  if (!active_) {
    ROS_ERROR("kinematics not active");
    return false;
  }

  KDL::Frame p_out;
  KDL::JntArray jnt_pos_in;
  geometry_msgs::PoseStamped pose;
  tf::Stamped<tf::Pose> tf_pose;

  jnt_pos_in.resize(dimension_);
  for (int i = 0; i < dimension_; i++) {
    jnt_pos_in(i) = joint_angles[i];
  }

  poses.resize(link_names.size());

  bool valid = true;
  for (unsigned int i = 0; i < poses.size(); i++) {
    ROS_DEBUG("End effector index: %d", pentaxis_arm_fk_solver_->getSegmentIndex(link_names[i]));
    if (pentaxis_arm_fk_solver_->JntToCart(jnt_pos_in, p_out, pentaxis_arm_fk_solver_->getSegmentIndex(link_names[i])) >= 0) {
      tf::poseKDLToMsg(p_out, poses[i]);
    } else {
      ROS_ERROR("Could not compute FK for %s", link_names[i].c_str());
      valid = false;
    }
  }

  return valid;
}

const std::vector<std::string>& PentaxisArmKinematicsPlugin::getJointNames() const
{
  if(!active_)
  {
    ROS_ERROR("kinematics not active");
  }
  return ik_solver_info_.joint_names;
}

const std::vector<std::string>& PentaxisArmKinematicsPlugin::getLinkNames() const
{
  if(!active_)
  {
    ROS_ERROR("kinematics not active");
  }
  return fk_solver_info_.link_names;
}

void
PentaxisArmKinematicsPlugin::desiredPoseCallback(
    const KDL::JntArray& jnt_array,
    const KDL::Frame& ik_pose,
    moveit_msgs::MoveItErrorCodes& error_code) const
{
  std::vector<double> ik_seed_state;
  ik_seed_state.resize(dimension_);
  int int_error_code;

  for (int i = 0; i < dimension_; i++) {
    ik_seed_state[i] = jnt_array(i);
  }

  geometry_msgs::Pose ik_pose_msg;
  tf::poseKDLToMsg(ik_pose, ik_pose_msg);

  desiredPoseCallback_(ik_pose_msg, ik_seed_state, error_code);
}


void PentaxisArmKinematicsPlugin::jointSolutionCallback(
    const KDL::JntArray& jnt_array,
    const KDL::Frame& ik_pose,
    moveit_msgs::MoveItErrorCodes& error_code) const
{
  std::vector<double> ik_seed_state;
  ik_seed_state.resize(dimension_);
  int int_error_code;

  for (int i = 0; i < dimension_; i++) {
    ik_seed_state[i] = jnt_array(i);
  }

  geometry_msgs::Pose ik_pose_msg;
  tf::poseKDLToMsg(ik_pose, ik_pose_msg);

  solutionCallback_(ik_pose_msg, ik_seed_state, error_code);
}

} /* namespace pentaxis_kinematics */

