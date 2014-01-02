kuka_youbot
===========

Description, controller, MoveIt configuration, and demo files for the Kuka YouBot.

This is the code that [Service Robotics & Technologies](http://srtlabs.com) is using  for its submission to Kuka's [Innovation in Mobile Manipulation Award](http://www.kuka-labs.com/en/network/innovationaward/) competition. Most of this code comes from [WPI-RAIL](https://github.com/WPI-RAIL) and is only slightly modified.

This software is being actively developend on the YouBot's onboard computer running ROS Hydro on Ubuntu 13.04

The following packages must also be installed on your ROS system in order for all this to work:
- [brics_actuator](https://github.com/WPI-RAIL/brics_actuator)
- [hokuyo_node](https://github.com/ros-drivers/hokuyo_node)
- [octomap_mapping](https://github.com/OctoMap/octomap_mapping/tree/hydro-devel)
- [octomap_ros](https://github.com/OctoMap/octomap_ros/tree/hydro-devel)
- [joy](https://github.com/ros-drivers/joystick_drivers) - required by youbot_teleop. I normally install via "sudo apt-get install ros-hydro-joy"
- [MoveIt!](http://moveit.ros.org/wiki/MoveIt!) - I normally install via "sudo apt-get install ros-hydro-moveit-full"
- [MoveIt Commander](https://github.com/ros-planning/moveit_commander) - required only for higher level interfacing with MoveIt
- GREG WAS HERE! :-)
