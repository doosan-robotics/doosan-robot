cmake_minimum_required(VERSION 3.5)
project(dsr_control)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  dsr_msgs
  controller_manager
  hardware_interface
  serial
  roscpp
  actionlib
)

SET( COMMON_INC_FILES
  #../../../common/include
  ../common/include
)
SET( COMMON_LIB_FILES
  ../common/lib
)

SET( SERIAL_SRC_FILES
  ../common/src/dsr_serial.cpp
)
catkin_package(
  INCLUDE_DIRS include ${COMMON_INC_FILES}
  LIBRARIES dsr_control
  CATKIN_DEPENDS dsr_msgs controller_manager hardware_interface roscpp serial
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${COMMON_INC_FILES}
)

SET( SRC_FILES
  ${SERIAL_SRC_FILES}
  src/dsr_hw_interface.cpp
  src/dsr_control_node.cpp
)

LINK_LIBRARIES(DRFL PocoFoundation PocoNet)

if($ENV{ROS_DISTRO} STREQUAL "kinetic")
  # check OS 32bit or 64bit ?  
  if("${CMAKE_SIZEOF_VOID_P}" EQUAL "8")
    LINK_DIRECTORIES ("${COMMON_LIB_FILES}/$ENV{ROS_DISTRO}/x86_64")
  else()
    LINK_DIRECTORIES ("${COMMON_LIB_FILES}/$ENV{ROS_DISTRO}/x86_32")
  endif()
elseif($ENV{ROS_DISTRO} STREQUAL "melodic")
  LINK_DIRECTORIES ("${COMMON_LIB_FILES}/$ENV{ROS_DISTRO}/x86_64")
elseif($ENV{ROS_DISTRO} STREQUAL "noetic")
  LINK_DIRECTORIES ("${COMMON_LIB_FILES}/$ENV{ROS_DISTRO}/x86_64")
else()
    message("Unknown ROS distro:")
    message($ENV{ROS_DISTRO})
endif()

add_executable(dsr_control_node ${SRC_FILES})
add_dependencies(dsr_control_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(dsr_control_node ${catkin_LIBRARIES})
target_link_libraries(dsr_control_node DRFL PocoFoundation PocoNet)



install(TARGETS dsr_control_node
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

add_library(dsr_control
  ${SERIAL_SRC_FILES}
  src/dsr_hw_interface.cpp
)

target_include_directories(dsr_control
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../common/include
)

target_link_libraries(dsr_control 
  PUBLIC
    DRFL 
    PocoFoundation 
    PocoNet
    ${catkin_LIBRARIES} 
)


install(TARGETS dsr_control
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

