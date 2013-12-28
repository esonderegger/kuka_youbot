cmake_minimum_required(VERSION 2.4.6)
project(youbot_navigation_common)

## check required prerequisites
SET(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/external/cmake_modules")
SET(Boost_ADDITIONAL_VERSIONS "1.39" "1.39.0" "1.40" "1.40.0" "1.41" "1.41.0" "1.42" "1.42.0" "1.43" "1.43.0" "1.44" "1.44.0" "1.45" "1.45.0" "1.46" "1.46.0")

find_package(catkin REQUIRED COMPONENTS roscpp geometry_msgs hokuyo_node)

catkin_package()

add_executable(lowpass_filter src/lowpass_filter.cpp)
target_link_libraries(lowpass_filter
   ${catkin_LIBRARIES}
)

