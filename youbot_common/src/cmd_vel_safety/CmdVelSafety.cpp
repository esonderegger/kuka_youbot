///

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/GridCells.h"
#include "tf/transform_listener.h"
#include "tf/transform_datatypes.h"

#include <math.h>
#include <unistd.h>

#include <boost/shared_ptr.hpp>

using namespace std;

double min_spd = 0.075; //any speed lower than this is allowed
double max_spd = 1.5; //maximum speed to be sent as a command vel

double timestep = 0.2; //how many seconds to estimate into the future

double stop_radius = 0.5;
double inflation_radius=1.5;

bool has_grid=false;
nav_msgs::GridCells obstacles;

ros::Publisher cmd_vel_pub;

boost::shared_ptr<tf::TransformListener> tf_listener;

double getMinimumDistance(tf::Vector3 position, nav_msgs::GridCells grid) {

  double distance_sq=-1;
  int size = grid.cells.size();
  double my_x=position.getX(), my_y=position.getY();
  double stop_radius_sq = stop_radius*stop_radius;
  for (int i=0; i<size; i++) {
    double d_sq = pow(grid.cells[i].x-my_x,2)+pow(grid.cells[i].y-my_y,2);
    if (distance_sq==-1 || d_sq<distance_sq) distance_sq=d_sq;
    if (d_sq<=stop_radius_sq) return 0;
  }

  return sqrt(distance_sq);
}

double getSafeVelocityLimit(tf::Vector3 vel) {
  //get current position + orientation
  //   by listening to the transform from obstacles->header->frame_id to /base_link
  tf::StampedTransform transform;
  try {
    tf_listener->lookupTransform(
        obstacles.header.frame_id,
        "/base_link",
        ros::Time(0),
        transform);
  }
  catch (tf::TransformException ex) {
    ROS_ERROR("Couldn't get current position: %s",ex.what());
    return min_spd;
  }
  //estimate future position, current_position+cmd_vel*sometimeconst*orientation
  tf::Vector3 future_pos = transform.getOrigin();
  tf::Vector3 v = vel.rotate(tf::Vector3(0,0,1),tf::getYaw(transform.getRotation()));
  future_pos=future_pos+timestep*v;
  //get minimum distance of future position
  if (!has_grid) {
    ROS_ERROR("Haven't received a grid yet so speed will be fully limited");
    return min_spd;
  }
  double mindist = getMinimumDistance(future_pos,obstacles);
  ROS_DEBUG("got minimum distance %f",mindist);
  //calculate velocity scale based on mindist of futurepos 
  //   (minspd@<=minrad --lerp--> 1@>=maxrad)
  double speed_limit;
  if (mindist>inflation_radius) speed_limit=max_spd;
  else if (mindist<=stop_radius) speed_limit=min_spd;
  else speed_limit = min(min_spd+(max_spd-min_spd)*(mindist-stop_radius)/(inflation_radius-stop_radius),max_spd);
  //return it
  ROS_DEBUG_STREAM("  computed velocity scale: "<<speed_limit);
  return speed_limit;
}

void obstaclesCallback(const nav_msgs::GridCells::ConstPtr& msg)
{
  if (!has_grid) ROS_INFO("Got first grid");
  else ROS_DEBUG("Got a grid");
  obstacles = *msg;
  has_grid=true;
}

void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
  geometry_msgs::Twist vel = *msg;
  ROS_DEBUG_STREAM("Got unsafe command vel (" 
      << vel.linear.x << ", "
      << vel.linear.y << ", " 
      << vel.angular.z << ")" );
  //put it in a tf vector for convenience
  tf::Vector3 v;
  v.setX(vel.linear.x);
  v.setY(vel.linear.y);
  //find the speed
  double speed = v.length();
  //find the speed limit
  double speed_limit = getSafeVelocityLimit(v);
  //limit the speed if needed
  if (speed_limit<speed) {
    v=v.normalized()*speed_limit;
    vel.linear.x=v.getX();
    vel.linear.y=v.getY();
  }
  //publish the limited speed
  cmd_vel_pub.publish(vel);
  ROS_DEBUG_STREAM("Sent safe command vel (" 
      << vel.linear.x << ", "
      << vel.linear.y << ", " 
      << vel.angular.z << ")" );
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "cmd_vel_safety");

  ros::NodeHandle n;

  n.getParam("/move_base_node/local_costmap/inflation_radius",stop_radius);
  ROS_INFO("Using obstacle radius of %lf",stop_radius);
  inflation_radius=max(stop_radius*1.5,stop_radius+0.2);

  tf_listener.reset(new tf::TransformListener());

  ros::Subscriber cmd_vel_sub;
  ROS_INFO("Subscribing to cmd_vel_unsafe");
  cmd_vel_sub = n.subscribe("cmd_vel_unsafe", 1, cmdVelCallback);

  ros::Subscriber grid_sub;
  ROS_INFO("Subscribing to /move_base_node/local_costmap/obstacles");
  grid_sub = n.subscribe("/move_base_node/global_costmap/obstacles", 1, obstaclesCallback);

  ROS_INFO("Advertising safe command velocity publisher");
  cmd_vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel",1);

  ros::spin();
}

