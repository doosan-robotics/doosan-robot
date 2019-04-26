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
  serial
)

catkin_package(
#   INCLUDE_DIRS include
#  LIBRARIES dr_example
   CATKIN_DEPENDS roscpp std_msgs serial
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
SET( SRC_FILES_SINGLE_ROBOT_BASIC
  ${COMMON_SRC_FILES}
  src/basic/single_robot_basic.cpp
)

#---- simple src --------------------------
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
SET( SRC_FILES_SINGLE_ROBOT_SIMPLE
  ${COMMON_SRC_FILES}
  src/simple/single_robot_simple.cpp
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

add_executable(single_robot_basic ${SRC_FILES_SINGLE_ROBOT_BASIC})
add_dependencies(single_robot_basic ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(single_robot_basic ${catkin_LIBRARIES})

#----- simple ---------------------------------------------------------------------------------------------
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

add_executable(single_robot_simple ${SRC_FILES_SINGLE_ROBOT_SIMPLE})
add_dependencies(single_robot_simple ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(single_robot_simple ${catkin_LIBRARIES})
