/*!
 * \file youbot_joy_teleop.cpp
 * \brief Allows for control of the KUKA youBot with a joystick.
 *
 * youbot_joy_teleop creates a ROS node that allows the control of a KUKA youBot with a joystick.
 * This node listens to a /joy topic and sends messages to the /cmd_vel topic. Arm control is currently unimplemented.
 *
 * \author Russell Toris, WPI - rctoris@wpi.edu
 * \date May 21, 2013
 */

#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <youbot_teleop/youbot_joy_teleop.h>

ros::Time T;
bool receivedmsg = false;

using namespace std;

youbot_joy_teleop::youbot_joy_teleop()
{
  // create the ROS topics
  cmd_vel = node.advertise < geometry_msgs::Twist > ("cmd_vel", 10);
  joy_sub = node.subscribe < sensor_msgs::Joy > ("joy", 10, &youbot_joy_teleop::joy_cback, this);

  ROS_INFO("youBot Joystick Teleop Started");
}

void youbot_joy_teleop::joy_cback(const sensor_msgs::Joy::ConstPtr& joy)
{
  if(!receivedmsg)
  {
    receivedmsg = true; 
  }
  T = ros::Time::now();

  // create the twist message
  geometry_msgs::Twist twist;
  // left joystick controls the linear movement
  twist.linear.x = joy->axes.at(1);
  twist.linear.y = joy->axes.at(0);
  twist.linear.z = 0;
  // right joystick controls the angular movement
  twist.angular.x = 0;
  twist.angular.y = 0;
  twist.angular.z = joy->axes.at(2);
  // send the twist command
  cmd_vel.publish(twist);
}

void youbot_joy_teleop::joy_check()
{
    if( (receivedmsg) && ( (ros::Time::now().toSec() - T.toSec() ) > .15) )
    {
      geometry_msgs::Twist zero;
      cmd_vel.publish(zero);
    }
}

int main(int argc, char **argv)
{
  // initialize ROS and the node
  ros::init(argc, argv, "youbot_joy_teleop");

  // initialize the joystick controller
  youbot_joy_teleop controller;

  // continue until a ctrl-c has occurred
  while(ros::ok())
  {
    controller.joy_check();
    
    ros::spinOnce();
  }
  //ros::spin();
}
