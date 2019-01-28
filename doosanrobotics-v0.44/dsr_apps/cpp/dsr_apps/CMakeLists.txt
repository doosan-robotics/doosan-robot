cmake_minimum_required(VERSION 2.8.3)
project(dsr_apps_cpp)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  dsr_msgs  
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES dr_example
   CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

SET( COMMON_INC_FILES
  ../../../common/include
)
SET( COMMON_SRC_FILES
  ../../../common/src/dsr_robot.cpp
)

include_directories(
  include  
  ${catkin_INCLUDE_DIRS}
  ${COMMON_INC_FILES}
)

SET( SRC_FILES
  ${COMMON_SRC_FILES}
  src/app_watch.cpp
)

add_executable(app_watch ${SRC_FILES})

add_dependencies(app_watch ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(app_watch ${catkin_LIBRARIES})
