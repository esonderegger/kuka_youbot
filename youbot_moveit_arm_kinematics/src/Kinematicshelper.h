/*
 * Kinematicshelper.h
 *
 *  Created on: Oct 29, 2013
 *      Author: raha
 */

#ifndef KINEMATICSHELPER_H_
#define KINEMATICSHELPER_H_

#include <urdf/model.h>
#include <Eigen/Core>
#include <kdl/chainiksolver.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <moveit_msgs/KinematicSolverInfo.h>
namespace youbot_kinematics
{

namespace kinematics_helper
{

Eigen::Matrix4f KDLToEigenMatrix(const KDL::Frame &p);
double  computeEuclideanDistance(const std::vector<double> &array_1, const KDL::JntArray &array_2);
double  computeEuclideanDistance(const KDL::JntArray &array_1, const KDL::JntArray &array_2);
void    getKDLChainInfo(const KDL::Chain &chain, moveit_msgs::KinematicSolverInfo &chain_info);
bool    getKDLChain(const urdf::ModelInterface& model, const std::string &root_name,
                      const std::string &tip_name, KDL::Chain &kdl_chain);

} /* namespace kinematics_helper */

} /* namespace youbot_kinematics */

#endif /* KINEMATICSHELPER_H_ */
