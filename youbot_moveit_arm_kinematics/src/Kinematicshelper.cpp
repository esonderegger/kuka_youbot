/*
 * Kinematicshelper.cpp
 *
 *  Created on: Oct 29, 2013
 *      Author: raha
 */

#include "Kinematicshelper.h"
#include "ros/console.h"

namespace youbot_kinematics
{

namespace kinematics_helper
{

bool
getKDLChain(const urdf::ModelInterface& model, const std::string &root_name,
            const std::string &tip_name, KDL::Chain &kdl_chain)
{
  // create robot chain from root to tip
  KDL::Tree tree;
  if (!kdl_parser::treeFromUrdfModel(model, tree))
  {
    ROS_ERROR("Could not initialize tree object");
    return false;
  }
  if (!tree.getChain(root_name, tip_name, kdl_chain))
  {
    ROS_ERROR_STREAM("Could not initialize chain object for base " << root_name
        << " tip " << tip_name);
    return false;
  }
  return true;
}

Eigen::Matrix4f
KDLToEigenMatrix(const KDL::Frame &p)
{
  Eigen::Matrix4f b = Eigen::Matrix4f::Identity();
  for (int i = 0; i < 3; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      b(i, j) = p.M(i, j);
    }
    b(i, 3) = p.p(i);
  }
  return b;
}

double
computeEuclideanDistance(const std::vector<double> &array_1,
                         const KDL::JntArray &array_2)
{
  double distance = 0.0;
  for (int i = 0; i < (int) array_1.size(); i++)
  {
    distance += (array_1[i] - array_2(i)) * (array_1[i] - array_2(i));
  }
  return sqrt(distance);
}

double
computeEuclideanDistance(
                         const KDL::JntArray &array_1,
                         const KDL::JntArray &array_2)
{
  double distance = 0.0;

  for (unsigned int i = 0; i < array_1.rows(); i++)
  {
    distance += (array_1(i) - array_2(i)) * (array_1(i) - array_2(i));
  }

  return sqrt(distance);
}

void
getKDLChainInfo(const KDL::Chain &chain,
                moveit_msgs::KinematicSolverInfo &chain_info)
{
  int i = 0; // segment number
  while (i < (int) chain.getNrOfSegments())
  {
    chain_info.link_names.push_back(chain.getSegment(i).getName());
    i++;
  }
}

} /* namespace kinematics_helper */

} /* namespace youbot_kinematics */
