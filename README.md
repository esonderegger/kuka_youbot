kuka_youbot
===========

Description, controller, MoveIt configuration, and demo files for the Kuka YouBot.

This is the code that [Service Robotics & Technologies](http://srtlabs.com) is using  for its submission to Kuka's [Innovation in Mobile Manipulation Award](http://www.kuka-labs.com/en/network/innovationaward/) competition. Most of this code comes from [WPI-RAIL](https://github.com/WPI-RAIL) and is only slightly modified.

Update!
-------

I've decided to create separate repositories for the component packages that used to live here. This allowed me to properly credit from where the code was forked, and to potentially contribute back to the main repositories. Also, some of the packages appeared to behave better when saved in the root level of catkin_ws

These are the component packages:
- [youbot_driver](https://github.com/esonderegger/youbot_driver) - At this point unmodified from the original
- [youbot_common](https://github.com/esonderegger/youbot_common) - a wrapper for the youbot driver, only barely modified to help build properly in catkin
- [youbot_description](https://github.com/esonderegger/youbot_description) - merges various youbot models available online, with our hokuyo upside down on the front and xtion on the gripper facing forward configuration
- [youbot_moveit_config](https://github.com/esonderegger/youbot_moveit_config) - fairly straightforward. doesn't attempt to do any IK
- [youbot_srt](https://github.com/esonderegger/youbot_srt) - the SRT specific stuff, like bringup.launch files and map files for my basement
- [youbot_navigation](https://github.com/esonderegger/youbot_navigation) - This doesn't work at all. Hopefully it will soon.

The following packages must also be installed on your ROS system in order for all this to work:
- [brics_actuator](https://github.com/WPI-RAIL/brics_actuator)
- [hokuyo_node](https://github.com/ros-drivers/hokuyo_node)
- [octomap_mapping](https://github.com/OctoMap/octomap_mapping/tree/hydro-devel)
- [octomap_ros](https://github.com/OctoMap/octomap_ros/tree/hydro-devel)
- [joy](https://github.com/ros-drivers/joystick_drivers) - required by youbot_teleop. I normally install via "sudo apt-get install ros-hydro-joy"
- [MoveIt!](http://moveit.ros.org/wiki/MoveIt!) - I normally install via "sudo apt-get install ros-hydro-moveit-full"
- [MoveIt Commander](https://github.com/ros-planning/moveit_commander) - required only for higher level interfacing with MoveIt
