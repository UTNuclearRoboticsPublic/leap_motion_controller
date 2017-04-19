cmake_minimum_required(VERSION 2.8.3)
project(leap_motion_controller)

## Find catkin macros and libraries
find_package(catkin REQUIRED
  roscpp
  rospy
  std_msgs
  geometry_msgs
  message_generation
)

## Use C++11
add_definitions(-std=c++11)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  Set.msg
  Hand.msg
  Finger.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
# INCLUDE_DIRS include
# LIBRARIES leap_motion_controller
 CATKIN_DEPENDS std_msgs geometry_msgs
# DEPENDS system_lib
)


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  $ENV{LEAP_SDK}/include/
)

## Declare a C++ executable
add_executable(leap_motion src/leap_motion.cpp)

## Add cmake target dependencies of the executable
add_dependencies(leap_motion leap_motion_controller_gencpp)

## Specify libraries to link a library or executable target against
target_link_libraries(leap_motion ${catkin_LIBRARIES} $ENV{LEAP_SDK}/lib/x64/libLeap.so)

