cmake_minimum_required(VERSION 2.8.3)

###########
## Build ##
###########

add_library(soem 
	ethercatbase.c 
  ethercatcoe.c 
  ethercatconfig.c 
  ethercatfoe.c 
  ethercatsoe.c 
  ethercatmain.c 
  ethercatprint.c 
  nicdrv.c 
  ethercatdc.c
)

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS soem 
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
