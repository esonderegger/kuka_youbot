#include <iostream>
#include <assert.h>
#include "ros/ros.h"
#include <stdio.h>
#include <unistd.h>
#include <termios.h>
#include "std_msgs/String.h"
#include "visualization_msgs/Marker.h"
#include "tf/transform_datatypes.h"
#include "tf/transform_listener.h"
#include "moveit_msgs/CollisionObject.h"
#include "moveit_msgs/PlanningScene.h"
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/collision_detection/world.h>
#include "geometry_msgs/Pose.h"
#include "shape_msgs/SolidPrimitive.h"
#include "control_msgs/FollowJointTrajectoryActionGoal.h"
#include "boost/thread.hpp"

#include <moveit/move_group_interface/move_group.h>

volatile bool trajectory_complete = false;
volatile char cancel = '0';
std::string goal_string = "0";

using namespace std;

void trajectory_execution(move_group_interface::MoveGroup *g, move_group_interface::MoveGroup::Plan *plan)
{
  trajectory_complete = g->execute(*plan);
}

void goal_callback(const control_msgs::FollowJointTrajectoryActionGoal::ConstPtr& msg)
{
  goal_string = msg->goal_id.id;
}

char getch(void)
{
	char ch;
	struct termios oldt;
	struct termios newt;
	tcgetattr(STDIN_FILENO, &oldt); /*store old settings */
	newt = oldt; /* copy old settings to new settings */
	newt.c_lflag &= ~(ICANON | ECHO); /* make one change to old settings in new settings */
	tcsetattr(STDIN_FILENO, TCSANOW, &newt); /*apply the new settings immediately */
	ch = getchar(); /* standard getchar call */
	tcsetattr(STDIN_FILENO, TCSANOW, &oldt); /*reapply the old settings */
	return ch; /*return received char */
}

void getch_check()
{
  cancel = '0';
  while(cancel != 'c')
  {
    boost::this_thread::interruption_point();
    cancel = getch();
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "youbot_moveit_to_pose", ros::init_options::AnonymousName);
  // start a ROS spinning thread
  ros::AsyncSpinner spinner(1);
  ros::Rate r(10);

  ros::NodeHandle n;

  ros::Publisher cancel_publisher = n.advertise<actionlib_msgs::GoalID>("/arm_1/arm_controller/follow_joint_trajectory/cancel", 1);
  ros::Subscriber goal_listener = n.subscribe("/arm_1/arm_controller/follow_joint_trajectory/goal",1,goal_callback);

  tf::TransformListener tf_listener;

  ROS_DEBUG("Starting youbot_moveit_teleop...");
  spinner.start();

  ROS_INFO("Connecting to move_group node for youbot arm...");
  // this connecs to a running instance of the move_group node
  move_group_interface::MoveGroup group("arm");

  ROS_DEBUG("\nSetting starting position to current position...");
  group.setStartStateToCurrentState();  //Sets the starting state for the move_group to be the current state of the robot

  if(!group.setNamedTarget(argv[1]))
  {
    ROS_ERROR("\nInvalid pose name provided.");
    return;
  }

  //Plan a motion path to the user-provided position
  move_group_interface::MoveGroup::Plan p;

  //If position is unreachable or a path is not possible, the user will be prompted to put in new information
  if(!group.plan(p))
  {
    ROS_ERROR("Destination is unreachable!");
    return;
  }

  trajectory_complete = false;

  goal_string = "0";
  //Execute will perform the trajectory.  It is non-blocking.
  boost::thread execute_thread(trajectory_execution, &group, &p);

  boost::thread read_cancel(getch_check);

  while(ros::ok())
  {
    ros::spinOnce();
    //Hitting 'c' mid trajectory will cancel the current trajectory
    if(cancel == 'c')
    {
      actionlib_msgs::GoalID g_id;
      g_id.id = goal_string;

      cancel_publisher.publish(g_id);
      ROS_WARN("Canceled the current trajectory!");
      ROS_DEBUG("Setting starting position to current position...");
group.setStartStateToCurrentState();  //Sets the starting state for the move_group to be the current state of the robot
      read_cancel.join();
      break;
    }

    if(trajectory_complete)
    {
      ROS_INFO("Trajectory completed.");
      execute_thread.join();
      read_cancel.interrupt();
      break;
    }
  }
}
