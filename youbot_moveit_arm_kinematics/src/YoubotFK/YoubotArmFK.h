/*
 * YoubotArmFK.h
 *
 *  Created on: Oct 29, 2013
 *      Author: raha
 */

#ifndef YOUBOTARMFK_H_
#define YOUBOTARMFK_H_

// ROS includes
#include <moveit_msgs/KinematicSolverInfo.h>
#include <moveit_msgs/GetKinematicSolverInfo.h>
#include <kdl/chainfksolver.hpp>
#include <urdf/model.h>

namespace youbot_kinematics
{

class YoubotArmFK : public KDL::ChainFkSolverPos
{
public:
    /**
     * Dtor.
     */
    virtual ~YoubotArmFK() {}


    /**
     * Perform the FK calculation.
     *
     * @param q_in Input joint coordinates.
     * @param p_out Reference to output cartesian pose.
     * @return < 0 if something went wrong.
     */
    virtual int JntToCart(const KDL::JntArray &q_in,
      KDL::Frame &p_out,
      int segmentNr = -1) = 0;

    /**
     * @brief A method to get chain information about the serial chain that the FK operates on
     * @param info This class gets populated with information about the joints that FK operates on, including joint names and limits.
     */
    virtual void getSolverInfo(moveit_msgs::KinematicSolverInfo &info) const = 0;

    /**
     * Get the index corresponding to the link name.
     */
    virtual int getSegmentIndex(const std::string &name) const = 0;
};

}/* namespace youbot_kinematics */


#endif /* YOUBOTARMFK_H_ */
