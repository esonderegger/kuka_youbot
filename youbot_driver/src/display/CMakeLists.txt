cmake_minimum_required(VERSION 2.8.3)

###########
## Build ##
###########

## Declare a cpp executable
add_executable(displayIpAddress DisplayIpAddress.cpp)

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS displayIpAddress
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
