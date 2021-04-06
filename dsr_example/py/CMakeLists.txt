cmake_minimum_required(VERSION 2.8.3)
project(dsr_example_py)

find_package(catkin REQUIRED COMPONENTS
# roscpp
  rospy
#  message_generation
# actionlib_msgs
  sensor_msgs
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

#############
## Install ##
#############

install(PROGRAMS scripts/demo/car.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS scripts/demo/dance_m1013.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(PROGRAMS scripts/demo/joy_ds4.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(PROGRAMS scripts/demo/joy_multi_xbox360.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(PROGRAMS scripts/demo/joy_xbox360.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(PROGRAMS scripts/demo/m1013x2_no_sync.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(PROGRAMS scripts/demo/m1013x2_sync.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(PROGRAMS scripts/demo/multi_robot.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(PROGRAMS scripts/gripper/object_pick_and_place_simple.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(PROGRAMS scripts/gripper/pick_and_place_simple.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(PROGRAMS scripts/gripper/real_pick_and_place_simple.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(PROGRAMS scripts/mobile/multi_robot_mobile.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(PROGRAMS scripts/mobile/single_robot_mobile_circle.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(PROGRAMS scripts/mobile/single_robot_mobile.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS scripts/simple/drl_reader_simple.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS scripts/simple/dsr_service_drl_simple.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS scripts/simple/dsr_service_io_simple.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS scripts/simple/dsr_service_modbus_simple.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS scripts/simple/dsr_service_motion_simple.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS scripts/simple/dsr_service_tool_simple.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS scripts/simple/dsr_simple_test.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS scripts/simple/jog_simple.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS scripts/simple/multi_robot_simple.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS scripts/simple/single_robot_simple.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
