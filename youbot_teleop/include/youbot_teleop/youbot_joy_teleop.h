/*!
 * \file youbot_joy_teleop.h
 * \brief Allows for control of the KUKA youBot with a joystick.
 *
 * youbot_joy_teleop creates a ROS node that allows the control of a KUKA youBot with a joystick.
 * This node listens to a /joy topic and sends messages to the /cmd_vel topic. Arm control is currently unimplemented.
 *
 * \author Russell Toris, WPI - rctoris@wpi.edu
 * \date May 21, 2013
 */

#ifndef YOUBOT_JOY_TELEOP_H_
#define YOUBOT_JOY_TELEOP_H_

#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>

/*!
 * \class youbot_joy_teleop
 * \brief Provides a bridge between the joy topic and the cmd_vel topic.
 *
 * The youbot_joy_teleop handles the translation between joystick commands and communication to the youBot's /cmd_vel topic.
 */
class youbot_joy_teleop
{
public:
  /*!
   * \brief Creates a youbot_joy_teleop.
   *
   * Creates a youbot_joy_teleop object that can be used control the KUKA youBot with a joystick.
   * ROS nodes, services, and publishers are created and maintained within this object.
   */
  youbot_joy_teleop();
  void joy_check();

private:
  /*!
   * \brief joy topic callback function.
   *
   * \param joy the message for the joy topic
   */
  void joy_cback(const sensor_msgs::Joy::ConstPtr& joy);

  ros::NodeHandle node; /*!< a handle for this ROS node */

  ros::Publisher cmd_vel; /*!< the cmd_vel topic */
  ros::Subscriber joy_sub; /*!< the joy topic */
};

/*!
 * Creates and runs the youbot_joy_teleop node.
 *
 * \param argc argument count that is passed to ros::init
 * \param argv arguments that are passed to ros::init
 * \return EXIT_SUCCESS if the node runs correctly
 */
int main(int argc, char **argv);

#endif
