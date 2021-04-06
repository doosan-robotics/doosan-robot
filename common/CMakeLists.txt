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

if($ENV{ROS_DISTRO} STREQUAL "kinetic")
    FIND_PACKAGE(gazebo REQUIRED)
elseif($ENV{ROS_DISTRO} STREQUAL "melodic")
    FIND_PACKAGE(gazebo REQUIRED)
elseif($ENV{ROS_DISTRO} STREQUAL "noetic")
    message("skip FIND_PACKAGE(gazebo REQUIRED) in noetic")
else()
    message("Unknown ROS distro:")
    message($ENV{ROS_DISTRO})
endif()

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

if($ENV{ROS_DISTRO} STREQUAL "kinetic")
    ADD_LIBRARY(gazebo_mimic_joint_plugin servicepack/plugin/robotiq_gazebo/kinetic/src/mimic_joint_plugin.cpp)
    TARGET_LINK_LIBRARIES(gazebo_mimic_joint_plugin
      ${catkin_LIBRARIES}
      ${GAZEBO_LIBRARIES})

    ADD_LIBRARY(gazebo_disable_link_plugin servicepack/plugin/robotiq_gazebo/kinetic/src/disable_link_plugin.cpp)
    TARGET_LINK_LIBRARIES(gazebo_disable_link_plugin
      ${catkin_LIBRARIES}
      ${GAZEBO_LIBRARIES})
elseif($ENV{ROS_DISTRO} STREQUAL "melodic")
    ADD_LIBRARY(gazebo_mimic_joint_plugin servicepack/plugin/robotiq_gazebo/melodic/src/mimic_joint_plugin.cpp)
    TARGET_LINK_LIBRARIES(gazebo_mimic_joint_plugin
      ${catkin_LIBRARIES}
      ${GAZEBO_LIBRARIES})

    ADD_LIBRARY(gazebo_disable_link_plugin servicepack/plugin/robotiq_gazebo/melodic/src/disable_link_plugin.cpp)
    TARGET_LINK_LIBRARIES(gazebo_disable_link_plugin
      ${catkin_LIBRARIES}
      ${GAZEBO_LIBRARIES})
elseif($ENV{ROS_DISTRO} STREQUAL "noetic")
    message("skip robotiq_gazebo_plugin in noetic")
else()
    message("Unknown ROS distro:")
    message($ENV{ROS_DISTRO})
endif()

if($ENV{ROS_DISTRO} STREQUAL "kinetic")
  # check OS 32bit or 64bit ?  
  if("${CMAKE_SIZEOF_VOID_P}" EQUAL "8")
    install(PROGRAMS 
      bin/DRCF/DRCF64_kinetic bin/DRCF/run_drcf_kinetic.sh
      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )
  else()
    install(PROGRAMS 
      bin/DRCF/DRCF32_kinetic bin/DRCF/run_drcf_kinetic.sh
      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )
  endif()
elseif($ENV{ROS_DISTRO} STREQUAL "melodic")
  install(PROGRAMS 
    bin/DRCF/DRCF64_melodic bin/DRCF/run_drcf_melodic.sh
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
elseif($ENV{ROS_DISTRO} STREQUAL "noetic")
  install(PROGRAMS 
    bin/DRCF/DRCF64_noetic bin/DRCF/run_drcf_noetic.sh
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
else()
    message("Unknown ROS distro:")
    message($ENV{ROS_DISTRO})
endif()

install(DIRECTORY
    imp bin/DRCF/config 
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)