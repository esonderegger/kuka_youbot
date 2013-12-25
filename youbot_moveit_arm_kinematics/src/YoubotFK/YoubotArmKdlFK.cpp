/*
 * YoubotArmKdlFK.cpp
 *
 *  Created on: Oct 29, 2013
 *      Author: raha
 */

#include <kdl/chainfksolverpos_recursive.hpp>
#include <ros/console.h>
#include <ros/assert.h>

#include "../SolverInfoProcessor.h"
#include "../Kinematicshelper.h"
#include "YoubotArmKdlFK.h"



namespace youbot_kinematics
{

YoubotArmKdlFK::YoubotArmKdlFK(const urdf::Model &robot_model,
                               const std::string &robot_description,
                               const std::string &root_name,
                               const std::string &tip_name)
{
  // create the KDL chain from the robot description
  if (!kinematics_helper::getKDLChain(robot_model, root_name, tip_name, _chain))
  {
    ROS_ERROR("Could not load KDL tree");
    ROS_ASSERT(false);
  }

  SolverInfoProcessor solver_info_processor(robot_model, tip_name, root_name);
  for (unsigned int i = 0; i < _solver_info.joint_names.size(); i++)
  {
    _solver_info.joint_names = solver_info_processor.getSolverInfo().joint_names;
  }
}

YoubotArmKdlFK::~YoubotArmKdlFK()
{
}

int
YoubotArmKdlFK::JntToCart(const KDL::JntArray &q_in,
                          KDL::Frame &p_out,
                          int segmentNr)
{
  KDL::ChainFkSolverPos_recursive fksolver(_chain);

  return fksolver.JntToCart(q_in, p_out, segmentNr);
}

void
YoubotArmKdlFK::getSolverInfo(moveit_msgs::KinematicSolverInfo &info) const
{
  info = _solver_info;
}

int
YoubotArmKdlFK::getSegmentIndex(const std::string &name) const
{
  int i = 0;
  while (i < (int) _chain.getNrOfSegments())
  {
    if (_chain.getSegment(i).getName() == name)
    {
      return i + 1;
    }
    i++;
  }
  return -1;
}

} /* namespace youbot_kinematics */
