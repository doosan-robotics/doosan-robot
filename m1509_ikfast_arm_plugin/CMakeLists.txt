cmake_minimum_required(VERSION 3.1)
project(m1509_ikfast_arm_plugin)

find_package(catkin REQUIRED COMPONENTS
  moveit_core
  pluginlib
  roscpp
  tf_conversions
)

include_directories(${catkin_INCLUDE_DIRS})

catkin_package()

include_directories(include src/solvers)

find_package(LAPACK REQUIRED)

macro(add_ikfast_plugin groupname)
  set(IKFAST_PLUGIN_GROUPNAME ${groupname})
  configure_file(src/m1509_GROUPNAME_moveit_ikfast_plugin.cpp.in ${CMAKE_BINARY_DIR}/m1509_${groupname}_moveit_ikfast_plugin.cpp)

  add_library(m1509_${groupname}_moveit_ikfast_plugin ${CMAKE_BINARY_DIR}/m1509_${groupname}_moveit_ikfast_plugin.cpp)
  target_link_libraries(m1509_${groupname}_moveit_ikfast_plugin ${catkin_LIBRARIES} ${LAPACK_LIBRARIES})
  set_target_properties(m1509_${groupname}_moveit_ikfast_plugin PROPERTIES
    CXX_STANDARD 11
    CXX_STANDARD_REQUIRED YES
    CXX_EXTENSIONS NO)

  install(TARGETS m1509_${groupname}_moveit_ikfast_plugin
          ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
          LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
          RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})
endmacro()

add_ikfast_plugin(arm)

install(
  FILES
  m1509_arm_moveit_ikfast_plugin_description.xml
  DESTINATION
  ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
