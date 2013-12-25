/*
 * YoubotArmKdlFK.h
 *
 *  Created on: Oct 29, 2013
 *      Author: raha
 */

#ifndef YOUBOTARMKDLFK_H_
#define YOUBOTARMKDLFK_H_

#include "YoubotArmFK.h"

namespace youbot_kinematics
{

class YoubotArmKdlFK: public YoubotArmFK
{
public:
  /**
   * Ctor.
   *
   * @brief Initialize the solver by providing a urdf::Model and a root and tip name.
   * @param robot_model A urdf::Model representation of the youBot robot model
   * @param robot_description The XML string that of a urdf::Model which represents the youBot robot model
   * @param root_name The root joint name of the arm
   * @param joint_name The tip joint name of the arm
   */
  YoubotArmKdlFK(const urdf::Model &robot_model,
                 const std::string &robot_description,
                 const std::string &root_name,
                 const std::string &tip_name);

  /**
   * Dtor.
   */
  virtual
  ~YoubotArmKdlFK();

  /**
   * @see ForwardKinematics::JntToCart
   */
  int
  JntToCart(const KDL::JntArray &q_in,
            KDL::Frame &p_out,
            int segmentNr = -1);

  /**
   * @see ForwardKinematics::getSolverInfo
   */
  void
  getSolverInfo(moveit_msgs::KinematicSolverInfo &response) const;

  /**
   * @see ForwardKinematics::getSegmentIndex
   */
  int
  getSegmentIndex(const std::string &name) const;

private:
  /**
   * Add a joint that is described by URDF to a kinematic chain description.
   * @param joint The joint in URDF description.
   * @param info The information about the IK solver which will be extended.
   */
  void
  addJointToChainInfo(boost::shared_ptr<const urdf::Joint> joint,
                      moveit_msgs::KinematicSolverInfo &info);

private:
  /**
   * The chain that the inverse kinematics is solved for.
   */
  KDL::Chain _chain;

  /**
   * Minimum joint limits.
   */
  std::vector<double> _min_angles;

  /**
   * Maximum joint limits.
   */
  std::vector<double> _max_angles;

  /**
   * Information about the IK solver.
   */
  moveit_msgs::KinematicSolverInfo _solver_info;
};

} /* namespace youbot_kinematics */

#endif /* YOUBOTARMKDLFK_H_ */
