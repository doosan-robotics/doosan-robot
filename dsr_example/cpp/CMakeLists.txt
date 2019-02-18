cmake_minimum_required(VERSION 2.8.3)
project(dsr_example_cpp)

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
#   INCLUDE_DIRS include
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

#---- basic src --------------------------
SET( SRC_FILES_BASIC_TEST
  ${COMMON_SRC_FILES}
  src/basic/dsr_basic_test.cpp
)
SET( SRC_FILES_MOTION_BASIC
  ${COMMON_SRC_FILES}
  src/basic/dsr_service_motion_basic.cpp
)
SET( SRC_FILES_MODBUS_BASIC
  ${COMMON_SRC_FILES}
  src/basic/dsr_service_modbus_basic.cpp
)
SET( SRC_FILES_DRL_BASIC
  ${COMMON_SRC_FILES}
  src/basic/dsr_service_drl_basic.cpp
)
SET( SRC_FILES_IO_BASIC
  ${COMMON_SRC_FILES}
  src/basic/dsr_service_io_basic.cpp
)

#---- simple src --------------------------
SET( SRC_SIMPLE_DANCE_M0617
  ${COMMON_SRC_FILES}
  src/simple/dance_m0617.cpp
)
SET( SRC_SIMPLE_DANCE_M1013
  ${COMMON_SRC_FILES}
  src/simple/dance_m1013.cpp
)

SET( SRC_SIMPLE_TEST
  ${COMMON_SRC_FILES}
  src/simple/dsr_simple_test.cpp
)
SET( SRC_FILES_MOTION_SIMPLE
  ${COMMON_SRC_FILES}
  src/simple/dsr_service_motion_simple.cpp
)
SET( SRC_FILES_MODBUS_SIMPLE
  ${COMMON_SRC_FILES}
  src/simple/dsr_service_modbus_simple.cpp
)
SET( SRC_FILES_IO_SIMPLE
  ${COMMON_SRC_FILES}
  src/simple/dsr_service_io_simple.cpp
)
SET( SRC_FILES_DRL_SIMPLE
  ${COMMON_SRC_FILES}
  src/simple/dsr_service_drl_simple.cpp
)

#---- multi_robot src --------------------------
SET( SRC_MULTI_M1013x2
  ${COMMON_SRC_FILES}
  src/multi_robot/m1013x2.cpp
)
SET( SRC_MULTI_M1013x2_NO_SYNC
  ${COMMON_SRC_FILES}
  src/multi_robot/m1013x2_no_sync.cpp
)
SET( SRC_MULTI_M1013x2_SYNC
  ${COMMON_SRC_FILES}
  src/multi_robot/m1013x2_sync.cpp
)
SET( SRC_MULTI_M1013x2_SYNC2
  ${COMMON_SRC_FILES}
  src/multi_robot/m1013x2_sync2.cpp
)

#---- mobile_robot src --------------------------
SET( SRC_MOBILE_M1013
  ${COMMON_SRC_FILES}
  src/mobile/m1013_on_mobile.cpp
)

#----- basic --------------------------------------------------------------------------------------------
add_executable(dsr_basic_test ${SRC_FILES_BASIC_TEST})
add_dependencies(dsr_basic_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(dsr_basic_test ${catkin_LIBRARIES})

add_executable(dsr_service_drl_basic ${SRC_FILES_DRL_BASIC})
add_dependencies(dsr_service_drl_basic ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(dsr_service_drl_basic ${catkin_LIBRARIES})

add_executable(dsr_service_io_basic ${SRC_FILES_IO_BASIC})
add_dependencies(dsr_service_io_basic ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(dsr_service_io_basic ${catkin_LIBRARIES})

add_executable(dsr_service_modbus_basic ${SRC_FILES_MODBUS_BASIC})
add_dependencies(dsr_service_modbus_basic ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(dsr_service_modbus_basic ${catkin_LIBRARIES})

add_executable(dsr_service_motion_basic ${SRC_FILES_MOTION_BASIC})
add_dependencies(dsr_service_motion_basic ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(dsr_service_motion_basic ${catkin_LIBRARIES})

#----- simple ---------------------------------------------------------------------------------------------
add_executable(dance_m0617 ${SRC_SIMPLE_DANCE_M0617})
add_dependencies(dance_m0617 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(dance_m0617 ${catkin_LIBRARIES})

add_executable(dance_m1013 ${SRC_SIMPLE_DANCE_M1013})
add_dependencies(dance_m1013 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(dance_m1013 ${catkin_LIBRARIES})

add_executable(dsr_service_drl_simple ${SRC_FILES_DRL_SIMPLE})
add_dependencies(dsr_service_drl_simple ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(dsr_service_drl_simple ${catkin_LIBRARIES})

add_executable(dsr_service_io_simple ${SRC_FILES_IO_SIMPLE})
add_dependencies(dsr_service_io_simple ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(dsr_service_io_simple ${catkin_LIBRARIES})

add_executable(dsr_service_modbus_simple ${SRC_FILES_MODBUS_SIMPLE})
add_dependencies(dsr_service_modbus_simple ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(dsr_service_modbus_simple ${catkin_LIBRARIES})

add_executable(dsr_service_motion_simple ${SRC_FILES_MOTION_SIMPLE})
add_dependencies(dsr_service_motion_simple ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(dsr_service_motion_simple ${catkin_LIBRARIES})

add_executable(dsr_simple_test ${SRC_SIMPLE_TEST})
add_dependencies(dsr_simple_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(dsr_simple_test ${catkin_LIBRARIES})

#----- multi robot ----------------------------------------------------------------------------------------
add_executable(m1013x2 ${SRC_MULTI_M1013x2})
add_dependencies(m1013x2 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(m1013x2 ${catkin_LIBRARIES})

add_executable(m1013x2_no_sync ${SRC_MULTI_M1013x2_NO_SYNC})
add_dependencies(m1013x2_no_sync ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(m1013x2_no_sync ${catkin_LIBRARIES})

add_executable(m1013x2_sync ${SRC_MULTI_M1013x2_SYNC})
add_dependencies(m1013x2_sync ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(m1013x2_sync ${catkin_LIBRARIES})

add_executable(m1013x2_sync2 ${SRC_MULTI_M1013x2_SYNC2})
add_dependencies(m1013x2_sync2 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(m1013x2_sync2 ${catkin_LIBRARIES})

#----- mobile robot ----------------------------------------------------------------------------------------
add_executable(m1013_on_mobile ${SRC_MOBILE_M1013})
add_dependencies(m1013_on_mobile ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(m1013_on_mobile ${catkin_LIBRARIES})