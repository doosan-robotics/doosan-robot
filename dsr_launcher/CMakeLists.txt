cmake_minimum_required(VERSION 2.8.3)
project(dsr_launcher)

find_package(catkin REQUIRED COMPONENTS roslaunch)

catkin_package()


install(
	DIRECTORY launch rviz worlds
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
