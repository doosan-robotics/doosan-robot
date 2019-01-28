cmake_minimum_required(VERSION 2.8.3)
project(dsr_control)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  dsr_msgs
  controller_manager
  hardware_interface
  roscpp
)
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS dsr_msgs controller_manager hardware_interface roscpp
)

SET( COMMON_INC_FILES
  ../../../common/include
)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${COMMON_INC_FILES}
)

SET( SRC_FILES
  src/dsr_hw_interface.cpp
  src/dsr_control_node.cpp
)

LINK_LIBRARIES(DRFL PocoFoundation PocoNet)
LINK_DIRECTORIES ("${PROJECT_SOURCE_DIR}/lib")

add_executable(dsr_control_node ${SRC_FILES})
add_dependencies(dsr_control_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(dsr_control_node ${catkin_LIBRARIES})
target_link_libraries(dsr_control_node DRFL PocoFoundation PocoNet)