cmake_minimum_required(VERSION 2.4.6)
project(youbot_description)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
#find_package(catkin REQUIRED COMPONENTS roscpp joy geometry_msgs brics_actuator moveit_ros_planning_interface tf)
find_package(catkin REQUIRED)

###################################################
## Declare things to be passed to other projects ##
###################################################

## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package()


#########################################
## Old CMakeLists.txt for Gazebo stuff ##
#########################################

#rosbuild_find_ros_package(pr2_controller_manager)

#OPTION(WITH_DEBUG_INFO "Publish debug info" ON)	#TODO:: move debug parameter to the yaml file

#IF(WITH_DEBUG_INFO)
#  ADD_DEFINITIONS(-DDEBUG_INFO)
#ENDIF(WITH_DEBUG_INFO)

#youbot base teleoperation experiment 
#rosbuild_add_executable(youbot_keyboard_teleop_experiment examples/keyboard_teleop_experiment.cpp)

#test for a joint position control of the youbot arm
#rosbuild_add_library(joint_position_control src/joint_position_control.cpp)

#youbot universal controller: joint space position control, joint velocity control, joint torque control
#rosbuild_add_library(youbot_universal_control src/youbot_universal_control.cpp src/joint_state_observer_gazebo.cpp)
#target_link_libraries(youbot_universal_control youbot_trajectory_action_server)


#Testing new library
#rosbuild_add_library( youbot_controllers
#  src/youbot_universal_control.cpp
#  src/joint_state_observer_gazebo.cpp
#  src/joint_position_control.cpp
#)
#target_link_libraries( youbot_controllers youbot_trajectory_action_server )



#rosbuild_add_executable(arm_joint_position_control_test examples/arm_joint_position_control_test.cpp)
#rosbuild_add_executable(youbot_controller_manager src/youbot_controller_manager.cpp)
#rosbuild_add_executable(arm_joint_velocity_control_test examples/arm_joint_velocity_control_test.cpp)
#rosbuild_add_executable(arm_joint_torque_control_test examples/arm_joint_torque_control_test.cpp)

#cartesian compliance control for the unified youbot arm and base system
#add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/src/20_sim_cartesian_compliance_control)
#rosbuild_add_library(cartesian_compliance_control src/cartesian_compliance_control.cpp)
#target_link_libraries(cartesian_compliance_control 20_sim_cartesian_compliance_control)
#rosbuild_add_executable(cartesian_compliance_control_test examples/cartesian_compliance_control_test.cpp)

#static joint states publisher
#rosbuild_add_executable(static_joint_publisher src/static_joint_publisher.cpp)

#depricated
#rosbuild_add_executable(base_command_republisher src/base_command_republisher.cpp)

