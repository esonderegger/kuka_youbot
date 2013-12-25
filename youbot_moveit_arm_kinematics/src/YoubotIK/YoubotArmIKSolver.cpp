/*
 * YoubotArmIKSolver.cpp
 *
 *  Created on: Oct 29, 2013
 *      Author: raha
 */

#include "YoubotArmIKSolver.h"
#include "../Kinematicshelper.h"

#include <kdl_parser/kdl_parser.hpp>
#include <string>



namespace youbot_kinematics
{

bool YoubotArmIKSolver::getCount(int &count,
                                 const int &max_count,
                                 const int &min_count)
{
  if(count > 0)
  {
    if(-count >= min_count)
    {
      count = -count;
      return true;
    }
    else if(count+1 <= max_count)
    {
      count = count+1;
      return true;
    }
    else
    {
      return false;
    }
  }
  else
  {
    if(1-count <= max_count)
    {
      count = 1-count;
      return true;
    }
    else if(count-1 >= min_count)
    {
      count = count -1;
      return true;
    }
    else
      return false;
  }
}

YoubotArmIKSolver::YoubotArmIKSolver(YoubotArmIK &ik,
                                       const urdf::ModelInterface &robot_model,
                                       const std::string &root_frame_name,
                                       const std::string &tip_frame_name,
                                       const double &search_discretization_angle,
                                       const int &free_angle):ChainIkSolverPos(), youbot_arm_ik_(ik)
{
  search_discretization_angle_ = search_discretization_angle;
  free_angle_ = free_angle;
  root_frame_name_ = root_frame_name;
  youbot_arm_ik_.getSolverInfo(_solver_info);
  active_ = true;
}

int YoubotArmIKSolver::CartToJnt(const KDL::JntArray& q_init,
                                 const KDL::Frame& p_in,
                                 KDL::JntArray &q_out)
{
  Eigen::Matrix4f b = kinematics_helper::KDLToEigenMatrix(p_in);

  std::vector<KDL::JntArray> solution_ik;
  youbot_arm_ik_.CartToJnt(q_init, p_in, solution_ik);


  if (solution_ik.empty()) return -1;

  double min_distance = 1e6;
  int min_index = -1;

  for (unsigned int i = 0; i < solution_ik.size(); i++)
  {
    ROS_DEBUG("Solution : %ud", i);

    for (unsigned int j = 0; j < solution_ik[i].rows(); j++)
    {
      ROS_DEBUG("%d: %f", j, solution_ik[i](j));
    }
    ROS_DEBUG(" ");
    ROS_DEBUG(" ");

    double tmp_distance = kinematics_helper::computeEuclideanDistance(solution_ik[i], q_init);
    if (tmp_distance < min_distance)
    {
      min_distance = tmp_distance;
      min_index = i;
    }
  }

  if (min_index > -1)
  {
    q_out.resize(solution_ik[min_index].rows());
    for (unsigned int i = 0; i < solution_ik[min_index].rows(); i++)
    {
      q_out(i) = solution_ik[min_index](i);
    }
    return 1;
  }
  else
  {
    return -1;
  }
}

int YoubotArmIKSolver::CartToJntSearch(const KDL::JntArray& q_in,
                                       const KDL::Frame& p_in,
                                       KDL::JntArray &q_out,
                                       const double &timeout)
{
  KDL::JntArray q_init = q_in;
  double initial_guess = q_init(free_angle_);

  ros::Time start_time = ros::Time::now();
  double loop_time = 0;
  int count = 0;


  int num_positive_increments = (int) ((_solver_info.limits[free_angle_].max_position
      - initial_guess) / search_discretization_angle_);
  int num_negative_increments = (int) ((initial_guess
      - _solver_info.limits[free_angle_].min_position)
      / search_discretization_angle_);
  ROS_DEBUG("%f %f %f %d %d \n\n",
            initial_guess,
            _solver_info.limits[free_angle_].max_position,
            _solver_info.limits[free_angle_].min_position,
            num_positive_increments,
            num_negative_increments);
  while(loop_time < timeout)
  {
    if(CartToJnt(q_init,p_in,q_out) > 0)
      return 1;
    if(!getCount(count,num_positive_increments,-num_negative_increments))
      return -1;
    q_init(free_angle_) = initial_guess + search_discretization_angle_ * count;
    ROS_DEBUG("%d, %f",count,q_init(free_angle_));
    loop_time = (ros::Time::now()-start_time).toSec();
  }
  if(loop_time >= timeout)
  {
    ROS_DEBUG("IK Timed out in %f seconds",timeout);
    return TIMED_OUT;
  }
  else
  {
    ROS_DEBUG("No IK solution was found");
    return NO_IK_SOLUTION;
  }
  return NO_IK_SOLUTION;
}

} /* namespace youbot_kinematics */
