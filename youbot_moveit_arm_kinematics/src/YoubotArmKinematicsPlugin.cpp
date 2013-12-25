/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Danish Technological Institute
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Rasmus Hasle Andersen */

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

#include "YoubotArmKinematicsPlugin.h"
#include "YoubotIK/YoubotArmAnalyticalIK.h"
#include "Kinematicshelper.h"
//#include "YoubotArmIKSolver.h"

using namespace KDL;
using namespace std;

PLUGINLIB_EXPORT_CLASS(youbot_kinematics::YoubotArmKinematicsPlugin, kinematics::KinematicsBase)

namespace youbot_kinematics {

YoubotArmKinematicsPlugin::YoubotArmKinematicsPlugin() :
    active_(false), dimension_(5), free_angle_(0)
{
}

bool
YoubotArmKinematicsPlugin::isActive()
{
  if (active_)
    return true;
  return false;
}

void
YoubotArmKinematicsPlugin::setRobotModel(
    boost::shared_ptr<urdf::ModelInterface>& robot_model)
{
  robot_model_ = robot_model;
}

bool YoubotArmKinematicsPlugin::initialize(const std::string& robot_description,
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

//  ROS_ERROR("node_handle.searchParam(urdf_xml,full_urdf_xml);");
//  node_handle.searchParam(urdf_xml,full_urdf_xml);
//  ROS_ERROR("urdf_xml: %s", urdf_xml.c_str());
//  ROS_ERROR("full_urdf_xml: %s", full_urdf_xml.c_str());

  if (!node_handle.getParam(robot_description, urdf_xml))
  {
    ROS_FATAL("Could not load the xml from parameter server: %s\n", urdf_xml.c_str());
    return false;
  }

//  node_handle.param(full_urdf_xml,xml_string,std::string());
//  ROS_ERROR("node_handle.param(full_urdf_xml,xml_string,std::string());");
//  ROS_ERROR("full_urdf_xml: %s", full_urdf_xml.c_str());
//  ROS_ERROR("xml_string: %s", xml_string.c_str());

  robot_model->initString(urdf_xml);

  robot_model_interface.reset(robot_model);

  setRobotModel(robot_model_interface);

  ROS_DEBUG("Loading KDL Tree");
  if(!kinematics_helper::getKDLChain(*robot_model_,base_frame_,tip_frame_,kdl_chain_))
  {
    active_ = false;
    ROS_ERROR("Could not load kdl tree");
  }

  //  jnt_to_pose_solver_.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain_));

  // setup the IK
  // _ik = boost::shared_ptr<InverseKinematics>(new ArmKdlInverseKinematics(robot_model, xml_string, base_name, tip_name));
  ik_.reset(
      new youbot_kinematics::YoubotArmAnalyticalIK(*robot_model_,
                                                   robot_description,
                                base_name,
                                tip_name));

  youbot_arm_ik_solver_.reset(
      new youbot_kinematics::YoubotArmIKSolver(
          *ik_,
          *robot_model_.get(),
          base_frame_,
          tip_frame_,
          search_discretization_,
          free_angle_));

  if(!youbot_arm_ik_solver_->active_)
  {
    ROS_ERROR("Could not load ik");
    active_ = false;
  }
  else
  {
    youbot_arm_ik_solver_->getSolverInfo(ik_solver_info_);
    kinematics_helper::getKDLChainInfo(kdl_chain_,fk_solver_info_);
    fk_solver_info_.joint_names = ik_solver_info_.joint_names;

    for(unsigned int i=0; i < ik_solver_info_.joint_names.size(); i++)
    {
      ROS_DEBUG("YoubotKinematics:: joint name: %s",ik_solver_info_.joint_names[i].c_str());
    }
    for(unsigned int i=0; i < ik_solver_info_.link_names.size(); i++)
    {
      ROS_DEBUG("YoubotKinematics can solve IK for %s",ik_solver_info_.link_names[i].c_str());
    }
    for(unsigned int i=0; i < fk_solver_info_.link_names.size(); i++)
    {
      ROS_DEBUG("YoubotKinematics can solve FK for %s",fk_solver_info_.link_names[i].c_str());
    }
    ROS_DEBUG("YoubotKinematics::active for %s",group_name.c_str());
    active_ = true;
  }
  return active_;
}

bool
YoubotArmKinematicsPlugin::getPositionIK(
    const geometry_msgs::Pose &ik_pose,
    const std::vector<double> &ik_seed_state,
    std::vector<double> &solution,
    moveit_msgs::MoveItErrorCodes &error_code,
    const kinematics::KinematicsQueryOptions &options) const
{
  ROS_ERROR("The function is not yet implemented: %s", BOOST_CURRENT_FUNCTION);
  return false;
}

bool YoubotArmKinematicsPlugin::searchPositionIK(const geometry_msgs::Pose &ik_pose,
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

  int ik_valid = youbot_arm_ik_solver_->CartToJntSearch(jnt_pos_in,
                                                     pose_desired,
                                                     jnt_pos_out,
                                                     timeout);
  if(ik_valid == youbot_kinematics::NO_IK_SOLUTION)
  {
    error_code.val = error_code.NO_IK_SOLUTION;
    ROS_DEBUG("No valid inverse kinematics found..");
    return false;
  }

  if(ik_valid >= 0)
  {
//    stringstream ss;
//    geometry_msgs::Pose p;
//    tf::poseKDLToMsg(pose_desired, p);
//    ss << "Q  : " << p.orientation;
//    ss << "Pos: " << p.position;
//    ROS_DEBUG("An IK solution was found for pose %s", ss.str().c_str());

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

bool YoubotArmKinematicsPlugin::searchPositionIK(const geometry_msgs::Pose &ik_pose,
                                                 const std::vector<double> &ik_seed_state,
                                                 double timeout,
                                                 const std::vector<double> &consistency_limit,
                                                 std::vector<double> &solution,
                                                 moveit_msgs::MoveItErrorCodes &error_code,
                                                 const kinematics::KinematicsQueryOptions &options) const
{
  ROS_ERROR("The function is not yet implemented: %s", BOOST_CURRENT_FUNCTION);
  return false;
}

bool YoubotArmKinematicsPlugin::searchPositionIK(const geometry_msgs::Pose &ik_pose,
                                                 const std::vector<double> &ik_seed_state,
                                                 double timeout,
                                                 std::vector<double> &solution,
                                                 const IKCallbackFn &solution_callback,
                                                 moveit_msgs::MoveItErrorCodes &error_code,
                                                 const kinematics::KinematicsQueryOptions &options) const
{
  ROS_ERROR("The function is not yet implemented: %s", BOOST_CURRENT_FUNCTION);
  return false;
}

bool
YoubotArmKinematicsPlugin::searchPositionIK(
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

bool YoubotArmKinematicsPlugin::getPositionFK(const std::vector<std::string> &link_names,
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
    ROS_DEBUG("End effector index: %d", youbot_arm_fk_solver_->getSegmentIndex(link_names[i]));
    if (youbot_arm_fk_solver_->JntToCart(jnt_pos_in, p_out, youbot_arm_fk_solver_->getSegmentIndex(link_names[i])) >= 0) {
      tf::poseKDLToMsg(p_out, poses[i]);
    } else {
      ROS_ERROR("Could not compute FK for %s", link_names[i].c_str());
      valid = false;
    }
  }

  return valid;
}

const std::vector<std::string>& YoubotArmKinematicsPlugin::getJointNames() const
{
  if(!active_)
  {
    ROS_ERROR("kinematics not active");
  }
  return ik_solver_info_.joint_names;
}

const std::vector<std::string>& YoubotArmKinematicsPlugin::getLinkNames() const
{
  if(!active_)
  {
    ROS_ERROR("kinematics not active");
  }
  return fk_solver_info_.link_names;
}

void
YoubotArmKinematicsPlugin::desiredPoseCallback(
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


void YoubotArmKinematicsPlugin::jointSolutionCallback(
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

} /* namespace youbot_kinematics */

