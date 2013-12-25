cmake_minimum_required(VERSION 2.8.3)
project(youbot_common)

set (orocos_kdl_DIR /opt/ros/hydro/share/orocos_kdl) #change the path to match your complied directory of orocos_kdl

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rosconsole message_generation nav_msgs std_srvs brics_actuator std_msgs roscpp actionlib tf control_msgs geometry_msgs youbot_driver diagnostic_msgs orocos_kdl) #move_base_msgs #pr2_controller_manager)
find_package(Boost REQUIRED COMPONENTS thread date_time filesystem system)
find_package(Eigen REQUIRED)

#######################################
## Declare ROS messages and services ##
#######################################

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  PowerBoardState.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)


###################################################
## Declare things to be passed to other projects ##
###################################################

## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES youbot_trajectory_action_server
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
)

link_directories(
  ${catkin_LIBRARY_DIRS}
)

set(CMAKE_BUILD_TYPE Release)

add_library(youbot_trajectory_action_server src/youbot_trajectory_action_server/joint_trajectory_action.cpp)
target_link_libraries(youbot_trajectory_action_server ${catkin_LIBRARIES} ${EIGEN_DEFINITIONS})

add_executable(youbot_wrapper
  src/youbot_wrapper.cpp 
  src/YouBotOODLWrapper.cpp
  src/YouBotConfiguration.cpp
  src/joint_state_observer.cpp
)

target_link_libraries(youbot_wrapper ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(youbot_wrapper youbot_wrapper_gencpp)

add_executable(youbot_arm_test 
  src/examples/youbot_arm_test.cpp 
)
target_link_libraries(youbot_arm_test ${catkin_LIBRARIES})

add_executable(youbot_2nd_arm_test 
  src/examples/youbot_2nd_arm_test.cpp 
)
target_link_libraries(youbot_2nd_arm_test ${catkin_LIBRARIES})

#add_executable(youbot_controller_manager 
#    src/youbot_controller_manager.cpp 
#)
#target_link_libraries(youbot_controller_manager ${catkin_LIBRARIES})

add_executable(youbot_network_estopper src/youbot_network_estopper/YoubotNetworkEstopper.cpp)
target_link_libraries(youbot_network_estopper ${catkin_LIBRARIES})

add_executable(heartbeat_generator src/youbot_network_estopper/HeartbeatGenerator.cpp)
target_link_libraries(heartbeat_generator ${catkin_LIBRARIES})

add_executable(cmd_vel_safety src/cmd_vel_safety/CmdVelSafety.cpp)
target_link_libraries(cmd_vel_safety ${catkin_LIBRARIES})

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS youbot_wrapper youbot_arm_test youbot_2nd_arm_test youbot_trajectory_action_server youbot_network_estopper heartbeat_generator #youbot_controller_manager
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Copy header files
install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp" PATTERN "*.h"
)

## Copy launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

