cmake_minimum_required(VERSION 2.8.3)
project(dsr_description)
find_package(catkin REQUIRED)
catkin_package()

install(DIRECTORY config launch meshes rviz urdf xacro
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
