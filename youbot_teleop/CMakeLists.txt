cmake_minimum_required(VERSION 2.8.3)
project(youbot_teleop)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
#find_package(catkin REQUIRED COMPONENTS roscpp joy geometry_msgs brics_actuator moveit_ros_planning_interface tf)
find_package(catkin REQUIRED COMPONENTS roscpp joy geometry_msgs brics_actuator tf)

###################################################
## Declare things to be passed to other projects ##
###################################################

## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package()

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
)

set(CMAKE_BUILD_TYPE Release)

add_executable(youbot_joy_teleop src/youbot_joy_teleop.cpp)
add_executable(youbot_teleop_arm src/youbot_teleop_arm.cpp)
#add_executable(youbot_moveit_teleop src/youbot_moveit_teleop.cpp)
#add_executable(pick_and_place_demo src/pick_and_place_demo.cpp)
target_link_libraries(youbot_teleop_arm ${catkin_LIBRARIES} ${Boost_LIBRARIES})
#target_link_libraries(youbot_moveit_teleop ${catkin_LIBRARIES} ${Boost_LIBRARIES})
target_link_libraries(youbot_joy_teleop ${catkin_LIBRARIES})
#target_link_libraries(pick_and_place_demo ${catkin_LIBRARIES})

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS youbot_joy_teleop
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(TARGETS youbot_teleop_arm
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
#install(TARGETS youbot_moveit_teleop
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)
install(PROGRAMS scripts/youbot_keyboard_teleop.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
