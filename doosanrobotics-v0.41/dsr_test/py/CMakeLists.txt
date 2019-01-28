cmake_minimum_required(VERSION 2.8.3)
project(dsr_test_py)

find_package(catkin REQUIRED COMPONENTS
# roscpp
  rospy
#  message_generation
# actionlib_msgs
  dsr_msgs  
)

# Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
#add_service_files(
#  FILES
#   #Service1.srv
#   #Service2.srv
#  WordCount.srv  
#)

## Generate actions in the 'action' folder
#add_action_files(  
#   DIRECTORY action
#   FILES 
#   Timer.action
#   #Action2.action
#)

# Generate added messages and services with any dependencies listed here
#generate_messages(
#  DEPENDENCIES
# actionlib_msgs
#  std_msgs  # Or other packages containing msgs
#)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES basics
#  CATKIN_DEPENDS rospy
#  DEPENDS system_lib
#  CATKIN_DEPENDS actionlib_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)