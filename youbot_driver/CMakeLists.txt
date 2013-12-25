cmake_minimum_required(VERSION 2.8.3)
project(youbot_driver)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS rosconsole roscpp)
find_package(Boost REQUIRED COMPONENTS thread date_time filesystem system regex)

###################################################
## Declare things to be passed to other projects ##
###################################################

## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES YouBotDriver soem
  CATKIN_DEPENDS rosconsole
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

link_directories(
  ${catkin_LIBRARY_DIRS}
)

add_definitions(-DUSE_ROS_LOGGING)
set(CMAKE_BUILD_TYPE Release)

## Simple Open EtherCAT Master Library
add_subdirectory(src/soem)

## Main youBot Driver
set(YOUBOT_DRIVER_SRC
  src/youbot/EthercatMaster.cpp
  src/youbot/EthercatMasterWithThread.cpp
  src/youbot/EthercatMasterWithoutThread.cpp
  src/generic/Logger.cpp
  src/generic/ConfigFile.cpp
  src/generic/PidController.cpp
  src/generic-joint/JointData.cpp
  src/generic-joint/JointTrajectory.cpp
  src/youbot/YouBotJoint.cpp
  src/youbot/JointLimitMonitor.cpp
  src/youbot/YouBotBase.cpp
  src/youbot/YouBotManipulator.cpp
  src/youbot/YouBotGripper.cpp
  src/youbot/YouBotGripperBar.cpp
  src/youbot/YouBotGripperParameter.cpp
  src/youbot/DataTrace.cpp
  src/youbot/GripperDataTrace.cpp
  src/youbot/YouBotJointParameter.cpp
  src/youbot/YouBotJointParameterReadOnly.cpp
  src/youbot/YouBotJointParameterPasswordProtected.cpp
  src/youbot/JointTrajectoryController.cpp
  src/base-kinematic/FourSwedishWheelOmniBaseKinematic.cpp
  src/base-kinematic/FourSwedishWheelOmniBaseKinematicConfiguration.cpp
)

add_library(YouBotDriver ${YOUBOT_DRIVER_SRC})
## Specify libraries to link a library or executable target against
target_link_libraries(YouBotDriver soem ${catkin_LIBRARIES} ${Boost_LIBRARIES})

## Test Program
add_subdirectory(src/testing)

## youBot display example application
add_subdirectory(src/display)

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS YouBotDriver
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

## Copy header files
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp" PATTERN "*.h"
)

## Copy config files
install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_ETC_DESTINATION}
)
install(DIRECTORY ld/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ld
  FILES_MATCHING PATTERN "*.conf"
) 
install(DIRECTORY udev_rules/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/udev_rules
  FILES_MATCHING PATTERN "*.rules"
)

