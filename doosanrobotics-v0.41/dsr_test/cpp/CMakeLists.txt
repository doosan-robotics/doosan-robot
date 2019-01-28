cmake_minimum_required(VERSION 2.8.3)
project(dsr_test_cpp)

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
  ../../common/include
)
SET( COMMON_SRC_FILES
  ../../common/src/dsr_robot.cpp
)

include_directories(
  include  
  ${catkin_INCLUDE_DIRS}
  ${COMMON_INC_FILES}
)

SET( SRC_FILES
  ${COMMON_SRC_FILES}
  src/dsr_test.cpp
)

#SET( SRC_FILES_DANCE_M0617_SIMPLE
#  ${COMMON_SRC_FILES}
#  src/dance_m0617.cpp
#)
#SET( SRC_FILES_DANCE_M1013_SIMPLE
#  ${COMMON_SRC_FILES}
#  src/dance_m1013.cpp
#)

SET( SRC_FILES_GAZEBO
  ${COMMON_SRC_FILES}
  src/dsr_gazebo.cpp
)

SET( SRC_FILES_R2
  ${COMMON_SRC_FILES}
  src/dsr_test_r2.cpp
)

SET( SRC_FILES_SIMPLE_R2
  ${COMMON_SRC_FILES}
  src/dsr_simple_test_r2.cpp
)

add_executable(dsr_test ${SRC_FILES})
add_dependencies(dsr_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(dsr_test ${catkin_LIBRARIES})

#add_executable(dance_m0617 ${SRC_FILES_DANCE_M0617_SIMPLE})
#add_dependencies(dance_m0617 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#target_link_libraries(dance_m0617 ${catkin_LIBRARIES})

#add_executable(dance_m1013 ${SRC_FILES_DANCE_M1013_SIMPLE})
#add_dependencies(dance_m1013 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#target_link_libraries(dance_m1013 ${catkin_LIBRARIES})

add_executable(dsr_gazebo ${SRC_FILES_GAZEBO})
add_dependencies(dsr_gazebo ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(dsr_gazebo ${catkin_LIBRARIES})

add_executable(dsr_test_r2 ${SRC_FILES_R2})
add_dependencies(dsr_test_r2 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(dsr_test_r2 ${catkin_LIBRARIES})

add_executable(dsr_simple_test_r2 ${SRC_FILES_SIMPLE_R2})
add_dependencies(dsr_simple_test_r2 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(dsr_simple_test_r2 ${catkin_LIBRARIES})
