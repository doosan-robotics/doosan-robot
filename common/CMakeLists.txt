cmake_minimum_required(VERSION 3.5)
project(common)
find_package(catkin REQUIRED)

catkin_package()

FIND_PACKAGE(catkin REQUIRED COMPONENTS
  roscpp
  gazebo_ros
  control_toolbox
  xmlrpcpp
)

#for noetic FIND_PACKAGE(gazebo REQUIRED)
FIND_PACKAGE(Boost REQUIRED COMPONENTS thread)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

CATKIN_PACKAGE(
  DEPENDS
    roscpp
    gazebo_ros
    control_toolbox
    xmlrpcpp
)

LINK_DIRECTORIES(
  ${GAZEBO_LIBRARY_DIRS}
)
INCLUDE_DIRECTORIES(
  ${Boost_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  include
)

#for noetic ADD_LIBRARY(gazebo_mimic_joint_plugin servicepack/plugin/robotiq_gazebo/src/mimic_joint_plugin.cpp)
#for noetic TARGET_LINK_LIBRARIES(gazebo_mimic_joint_plugin
#for noetic  ${catkin_LIBRARIES}
#for noetic  ${GAZEBO_LIBRARIES})

#for noetic ADD_LIBRARY(gazebo_disable_link_plugin servicepack/plugin/robotiq_gazebo/src/disable_link_plugin.cpp)
#for noetic TARGET_LINK_LIBRARIES(gazebo_disable_link_plugin
#for noetic  ${catkin_LIBRARIES}
#for noetic  ${GAZEBO_LIBRARIES})

install(PROGRAMS
    bin/DRCF/DRCF64 bin/DRCF/run_drcf.sh
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY
    imp bin/DRCF/config
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)