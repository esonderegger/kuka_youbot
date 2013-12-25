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
//planning_scene::PlanningScene* current_scene = NULL;


using namespace std;

void trajectory_execution(move_group_interface::MoveGroup *g, move_group_interface::MoveGroup::Plan *plan)
{
  trajectory_complete = g->execute(*plan);
}

void goal_callback(const control_msgs::FollowJointTrajectoryActionGoal::ConstPtr& msg)
{
  goal_string = msg->goal_id.id;
}

/*
void planning_scene_callback(const moveit_msgs::PlanningScene::ConstPtr& msg)
{
  current_scene.clone(msg);
}


void clear_scene()
{
  moveit_msgs::PlanningScene cleared_scene(current_scene);

  
  //  I can show you the world 
  //  Shining, shimmering, splendid 
  //  Tell me, princess, now when did 
  //  You last let your heart decide?

  //  I can open your eyes 
  //  Take you wonder by wonder 
  //  Over, sideways and under 
  //  On a magic carpet ride

  cleared_scene = current_scene;

  //A whole new wooooooooooooorld!
  collision_detection::World new_world;
  
  //A new fantastic point of view!
  cleared_scene.world = new_world;
}
*/

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
  ros::init(argc, argv, "youbot_moveit_teleop", ros::init_options::AnonymousName);
  // start a ROS spinning thread
  ros::AsyncSpinner spinner(1);
  ros::Rate r(10);

  ros::NodeHandle n;

  ros::Publisher marker_publisher = n.advertise<visualization_msgs::Marker>("goal_marker", 1);
  //ros::Publisher clear_publisher = n.advertise<moveit_msgs::CollisionObject>("/planning_scene", 1);
  ros::Publisher cancel_publisher = n.advertise<actionlib_msgs::GoalID>("/arm_1/arm_controller/follow_joint_trajectory/cancel", 1);
  ros::Subscriber goal_listener = n.subscribe("/arm_1/arm_controller/follow_joint_trajectory/goal",1,goal_callback);
  //ros::Subscriber planning_scene_listener = n.subscribe("/planning_scene",1,planning_scene_callback);

  tf::TransformListener tf_listener;

  cout << "\nStarting youbot_moveit_teleop..." << endl;
  spinner.start();

  cout << "Connecting to move_group node for youbot arm..." << endl;
  // this connecs to a running instance of the move_group node
  move_group_interface::MoveGroup group("arm");

  //group.setEndEffector("gripper_eef");
  //group.setEndEffectorLink("arm_link_5");

  std::string eef_link = group.getEndEffectorLink();
  std::string eef = group.getEndEffector();

  vector<string> states;

  robot_model::RobotModel temp = *(*group.getCurrentState()).getRobotModel();

  (*temp.getJointModelGroup("arm")).getKnownDefaultStates(states);

  //temp.robot_model::RobotModel::~RobotModel();

  cout << "\nEnd Effector Link for this group is: " << eef_link << endl;
  cout << "\nEnd Effector for this group is: " << eef << endl;

  double tolerance = .05;
  double orientationtolerance = 3.141592654;
  double positiontolerance = 0.05;

  //cout << "Setting goal tolerance";
  group.setGoalTolerance(tolerance);
  //group.setGoalOrientationTolerance(orientationtolerance);
  //group.setGoalPositionTolerance(positiontolerance);

  while(ros::ok())
  {

    visualization_msgs::Marker goalmarker;
    goalmarker.header.frame_id = "/odom";
    goalmarker.id = 0;
    goalmarker.type = goalmarker.MESH_RESOURCE;
    goalmarker.action = goalmarker.ADD;
    goalmarker.scale.x = 1;
    goalmarker.scale.y = 1;
    goalmarker.scale.z = 1;
    goalmarker.pose.position.x = 0;
    goalmarker.pose.position.y = 0;
    goalmarker.pose.position.z = 0;
    goalmarker.color.r = 0;
    goalmarker.color.g = 1;
    goalmarker.color.b = 0;
    goalmarker.color.a = 1;
    goalmarker.mesh_resource = "package://youbot_description/meshes/youbot_gripper/palm.dae";


    cout << "\nSetting starting position to current position..." << endl;
    group.setStartStateToCurrentState();  //Sets the starting state for the move_group to be the current state of the robot
    
    cout << "Select mode of control:\n\t1)\tTarget specific point\n\t2)\tControlled Marker Teleop\n\t3)\tJoint Poses\n\t4)\tExit" << endl;
    
    int choose = 0;

    cin >> choose;

    double xposition = 0;
    double yposition = 0;
    double zposition = 0;

    double roll = 0;
    double pitch = 0;
    double yaw = 0;

    geometry_msgs::Pose new_pose;

    if(choose == 1)
    {
      cout << "\n---------------------------------------------------------------------" << endl;

      cout << "Is your target frame relative to the arm or to the global frame (/odom)?\n\t1)\tarm\n\t2)\todom" << endl;

      int frame = 0;

      cin >> frame;

      cout << "\nPlease provide an x,y, and z position that the arm will move towards." << endl;

      cout << "\nPlease enter an x position for the arm: ";
      cin >> xposition; //The user-provided x-position for the end-effector
      cout << "Please enter a y position for the arm: ";
      cin >> yposition; //The user-provided y-position for the end-effector
      cout << "Please enter a z position for the arm: ";
      cin >> zposition; //The user-provided z-position for the end-effector
      cout << "\n\nYou chose the point (" << xposition << ", " << yposition << ", " << zposition << ") ";

      cout << "\nPlease provide a roll,pitch, and yaw orientation that the arm will move towards." << endl;

      cout << "\nPlease enter a roll for the arm: ";
      cin >> roll; //The user-provided x-position for the end-effector
      cout << "Please enter a pitch for the arm: ";
      cin >> pitch; //The user-provided y-position for the end-effector
      cout << "Please enter a yaw for the arm: ";
      cin >> yaw; //The user-provided z-position for the end-effector
      cout << "\n\nYou chose the point (" << xposition << ", " << yposition << ", " << zposition << ") with orientation (" << roll << ", " << pitch << ", " << yaw << ") ";

      new_pose.position.x = xposition;
      new_pose.position.y = yposition;
      new_pose.position.z = zposition;

      new_pose.orientation.x = tf::createQuaternionFromRPY(roll, pitch, yaw).getAxis().x();
      new_pose.orientation.y = tf::createQuaternionFromRPY(roll, pitch, yaw).getAxis().y();
      new_pose.orientation.z = tf::createQuaternionFromRPY(roll, pitch, yaw).getAxis().z();
      new_pose.orientation.w = tf::createQuaternionFromRPY(roll, pitch, yaw).getAxis().w();

      if(frame == 1)
      {     
        cout << "relative to the arm." << endl;
        ROS_INFO("Transforming arm coordinate frame to global (odom) coordinate frame.");

        tf::StampedTransform arm_relative_to_odom;

        try
        {
          tf_listener.lookupTransform("arm_link_0", "/odom",ros::Time(0), arm_relative_to_odom);
        }
        catch(tf::TransformException ex)
        {
          ROS_ERROR("%s",ex.what());
        }
 
        tf::Transform point_relative_to_odom;

        tf::Transform point_relative_to_arm;
        tf::Vector3 point_tf_origin;
      
        point_tf_origin.setX(xposition);
        point_tf_origin.setY(yposition);
        point_tf_origin.setZ(zposition);

        point_relative_to_arm.setOrigin(point_tf_origin);
    
        point_relative_to_odom = point_relative_to_arm * arm_relative_to_odom;

        xposition = point_relative_to_odom.getOrigin().getX();
        yposition = point_relative_to_odom.getOrigin().getY();
        zposition = point_relative_to_odom.getOrigin().getZ();
      }
      if(frame == 2)
      {
        cout << "relative to the global (odom) coordinate frame." << endl;
      }
      if((frame != 1)&&(frame != 2))
      {
        cout << "." << endl;
        ROS_WARN("Invalid frame provided.  Defaulting to global (odom) coordinate frame.");
      }

      goalmarker.pose.position.x = xposition;
      goalmarker.pose.position.y = yposition;
      goalmarker.pose.position.z = zposition;

      marker_publisher.publish(goalmarker);

    } //END OF CHOICE 1/////////////////////////////////////////////////





    if(choose == 2)
    {
      cout << "Generating initial marker at end effector." << endl;
      
      tf::StampedTransform tf_odom_gripper;

      try
      {
        tf_listener.lookupTransform("/odom","gripper_palm_link",ros::Time(0), tf_odom_gripper);
      }
      catch(tf::TransformException ex)
      {
        ROS_ERROR("%s",ex.what());
      }

      goalmarker.pose.position.x = tf_odom_gripper.getOrigin().getX();
      goalmarker.pose.position.y = tf_odom_gripper.getOrigin().getY();
      goalmarker.pose.position.z = tf_odom_gripper.getOrigin().getZ();
  
      //goalmarker.pose = goalmarker.pose * transform;

      cout << "To move the target marker along the x-axis, press a/d.\nTo move the target marker along the y-axis, press w/s.\nTo move the target marker along the z-axis, press t/g.\nTo confirm your target location, press 'o'." << endl;

      char keyboardInput = '0';

      while((keyboardInput != 'o')&&ros::ok())
      {
        keyboardInput = getch();
        
        if(keyboardInput == 'a')
          goalmarker.pose.position.x += 0.01;
        if(keyboardInput == 'd')
          goalmarker.pose.position.x -= 0.01;
        if(keyboardInput == 'w')
          goalmarker.pose.position.y += 0.01;
        if(keyboardInput == 's')
          goalmarker.pose.position.y -= 0.01;
        if(keyboardInput == 't')
          goalmarker.pose.position.z += 0.01;
        if(keyboardInput == 'g')
          goalmarker.pose.position.z -= 0.01;

        marker_publisher.publish(goalmarker);
      }

      tf::Transform goal_tf;
      tf::Transform tf_arm_goalmarker;

      tf::Vector3 goal_tf_origin;
    
      goal_tf_origin.setX(goalmarker.pose.position.x);
      goal_tf_origin.setY(goalmarker.pose.position.y);
      goal_tf_origin.setZ(goalmarker.pose.position.z);

      goal_tf.setOrigin(goal_tf_origin);

      tf::StampedTransform tf_arm_odom;

      try
      {
        tf_listener.lookupTransform("arm_link_0", "/odom",ros::Time(0), tf_arm_odom);
      }
      catch(tf::TransformException ex)
      {
        ROS_ERROR("%s",ex.what());
      }

      //arm->goal         //arm->odom       //odom->goal
      tf_arm_goalmarker = tf_arm_odom  *  goal_tf;

      double goalXToArmX = tf_arm_goalmarker.getOrigin().getX();
      double goalYToArmY = tf_arm_goalmarker.getOrigin().getY();
      double goalZToArmZ = tf_arm_goalmarker.getOrigin().getZ();

      cout << "Target selected: (" << goalXToArmX << ", " << goalYToArmY << ", " << goalZToArmZ << ")" << endl;

      xposition = goalmarker.pose.position.x;
      yposition = goalmarker.pose.position.y;
      zposition = goalmarker.pose.position.z;
    } //END OF CHOICE 2/////////////////////////////////////////////////////




    if(choose == 3)
    {
      /*
      
      ros::spinOnce();
      
      clear_scene();

      clear_publisher.publish(cleared_scene);
      */      
      
      /*
      moveit_msgs::CollisionObject TheApocalypse;

      TheApocalypse.id = "Barbra Streisand";
      TheApocalypse.operation = TheApocalypse.ADD;
      TheApocalypse.header.stamp = ros::Time::now();
      TheApocalypse.header.frame_id = "/odom";

      shape_msgs::SolidPrimitive boom;

      boom.type = boom.BOX;
      boom.dimensions.resize(3);
      boom.dimensions[0] = 10000;
      boom.dimensions[1] = 10000;
      boom.dimensions[2] = 10000;

      TheApocalypse.primitives.push_back(boom);

      geometry_msgs::Pose ground_zero;

      TheApocalypse.primitive_poses.push_back(ground_zero);

      clear_publisher.publish(TheApocalypse);
      */

      //group.pick("object_3");

      //continue;

      /*
      cout << "All remembered joint states: " << endl;
        
      for( map< string, vector< double > >::const_iterator ii= group.getRememberedJointValues().begin(); ii!= group.getRememberedJointValues().end(); ++ii)
      {
        cout << "\t" << (*ii).first << endl;
        for(int j = 0; j < (*ii).second.size(); j++)
        {
          cout << "\t\t\t" << (*ii).second.at(j) << endl;
        }
      }
      
      cout << "Current State is: \n" << group.getCurrentState();
      
      continue;     
      */
 
      cout << "Joint Pose Target Names: " << endl;

      for(vector<string>::iterator i= states.begin(); i!= states.end(); ++i)
      {
        cout << "\t" << (*i) << endl;
      }        

      string target;

      cout << "Specify name of joint pose target to reach: ";
      cin >> target;
      if(!group.setNamedTarget(target))
      {
        ROS_ERROR("\nInvalid pose name provided.  Please try again.");
        //temp.robot_model::RobotModel::~RobotModel();
        continue;
      }
    }

    if(choose == 4)
    {
      cout << "Exiting...";
      break;
    }

    if(choose != 3)
    {
      cout << "Setting Position target to the provided location: (" << xposition << ", " << yposition << ", " << zposition << ") with orientation: (" << roll << ", " << pitch<< ", " << yaw << ")" << endl;

      //Sets the target position for the move_group to the user-sepcified location
      group.setPoseTarget(new_pose);
     
      cout << "\nPose target is: \n" << group.getPoseTarget(eef).pose << endl;
    }
    cout << "\n\t\tJoint targets are:\tCurrent joint values are:" << endl;
    for(int i = 0; i < group.getJointValueTarget().getJointNames().size(); i++)
    {
      robot_state::JointState j = *group.getJointValueTarget().getJointStateVector().at(i);

      cout << group.getJointValueTarget().getJointNames()[i] << ":  " << j.getVariableValues().back() << "\t\t\t" << group.getCurrentJointValues().at(i) << endl;
     
    }

    cout << "\n";

    //Plan a motion path to the user-provided position
    move_group_interface::MoveGroup::Plan p;

    //If position is unreachable or a path is not possible, the user will be prompted to put in new information
    if(!group.plan(p))
    {
      ROS_ERROR("Destination is unreachable!");
      continue;
    }

    goalmarker.pose.position.x = xposition;
    goalmarker.pose.position.y = yposition;
    goalmarker.pose.position.z = zposition;

    marker_publisher.publish(goalmarker);

    cout << "Please press be sure to monitor the arm during its trajectory.  If you wish to cancel the trajectory during execution, please press 'c' during execution." << endl << "To start the trajectory, please press 's' and then enter: ";

    char enter = 0;

    cin >> enter;

    if(enter == 's')
    {
      trajectory_complete = false;

      cout << "~~Moving to requested position~~" << endl; 

      goal_string = "0";
      //Execute will perform the trajectory.  It is non-blocking.
      boost::thread execute_thread(trajectory_execution, &group, &p);

      //cout << "Am I really non-blocking?" << endl;
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
          cout << "Canceled the current trajectory!" << endl;
          cout << "Setting starting position to current position..." << endl;
    group.setStartStateToCurrentState();  //Sets the starting state for the move_group to be the current state of the robot
          read_cancel.join();
          break;
        }
        
        //ros::spinOnce();

        if(trajectory_complete)
        {
          ROS_INFO("Trajectory completed.");
          execute_thread.join();
          read_cancel.interrupt();
          break;
        }
      }
    }

    cout << "Hit Control + C to exit." << endl;
    r.sleep();
  }
}
