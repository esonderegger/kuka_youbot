/*
 * YoubotArmIKSolver.h
 *
 *  Created on: Oct 29, 2013
 *      Author: raha
 */

#ifndef YoubotArmIKSolverH_
#define YoubotArmIKSolverH_

#include <urdf/model.h>
#include <Eigen/Core>
#include <kdl/chainiksolver.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/tree.hpp>
//#include <pr2_arm_kinematics/pr2_arm_kinematics_utils.h>
#include <moveit_msgs/GetKinematicSolverInfo.h>
#include <moveit_msgs/PositionIKRequest.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf_conversions/tf_kdl.h>
//#include <arm_navigation_msgs/ArmNavigationErrorCodes.h>
//#include <youbot_arm_kinematics/inverse_kinematics.h>
#include "YoubotArmIK.h"

namespace youbot_kinematics
{

static const int NO_IK_SOLUTION = -1;
static const int TIMED_OUT = -2;

class YoubotArmIKSolver : public KDL::ChainIkSolverPos
{

public:

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /** @class
   *  @brief ROS/KDL based interface for the inverse kinematics of the Youbot arm
   *  @author Sachin Chitta <sachinc@willowgarage.com>
   *
   *  This class provides a KDL based interface to the inverse kinematics of the Youbot arm. It inherits from the KDL::ChainIkSolverPos class
   *  but also exposes additional functionality to return multiple solutions from an inverse kinematics computation.
   */
  YoubotArmIKSolver(YoubotArmIK &ik,
                    const urdf::ModelInterface &robot_model,
                    const std::string &root_frame_name,
                    const std::string &tip_frame_name,
                    const double &search_discretization_angle,
                    const int &free_angle);

  ~YoubotArmIKSolver(){};

  /**
   * @brief The Youbot inverse kinematics solver
   */
  YoubotArmIK &youbot_arm_ik_;

  /**
   * @brief Indicates whether the solver has been successfully initialized
   */
  bool active_;

  int CartToJnt(const KDL::JntArray& q_init,
                const KDL::Frame& p_in,
                KDL::JntArray& q_out);

  int CartToJntSearch(const KDL::JntArray& q_in,
                      const KDL::Frame& p_in,
                      KDL::JntArray &q_out,
                      const double &timeout);

  void getSolverInfo(moveit_msgs::KinematicSolverInfo &response)
  {
    youbot_arm_ik_.getSolverInfo(response);
  }

private:

  bool getCount(int &count, const int &max_count, const int &min_count);

  double search_discretization_angle_;

  int free_angle_;

  std::string root_frame_name_;
  moveit_msgs::KinematicSolverInfo _solver_info;
};

} /* namespace youbot_kinematics */

#endif /* YoubotArmIKSolverH_ */
