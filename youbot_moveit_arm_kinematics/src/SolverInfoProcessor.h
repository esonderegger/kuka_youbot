/*
 * SolverInfoProcessor.h
 *
 *  Created on: Oct 29, 2013
 *      Author: raha
 */

#ifndef SOLVERINFOPROCESSOR_H_
#define SOLVERINFOPROCESSOR_H_

#include <boost/shared_ptr.hpp>
#include <moveit_msgs/KinematicSolverInfo.h>
#include <urdf/model.h>

namespace youbot_kinematics
{

class SolverInfoProcessor
{
public:
  /**
   * Ctor.
   */
  SolverInfoProcessor(const urdf::ModelInterface &robot_model,
                      const std::string &tip_name,
                      const std::string &root_name);

  /**
   * Dtor.
   */
  virtual
  ~SolverInfoProcessor();

  /**
   * Get the extracted information.
   */
  moveit_msgs::KinematicSolverInfo
  getSolverInfo() const;

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
   * The extracted solver information.
   */
  moveit_msgs::KinematicSolverInfo _solver_info;

};

} /* namespace youbot_kinematics */

#endif /* SOLVERINFOPROCESSOR_H_ */
