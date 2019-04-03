cmake_minimum_required(VERSION 2.8.3)
project(dsr_launcher)

find_package(catkin REQUIRED COMPONENTS roslaunch)

catkin_package()

roslaunch_add_file_check(launch/single_robot_rviz.launch)
roslaunch_add_file_check(launch/single_robot_gazebo.launch)
roslaunch_add_file_check(launch/single_robot_rviz_gazebo.launch)
roslaunch_add_file_check(launch/multi_robot_rviz.launch)
roslaunch_add_file_check(launch/multi_robot_gazebo.launch)
roslaunch_add_file_check(launch/multi_robot_rviz_gazebo.launch)


install(
	DIRECTORY launch rviz worlds
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
