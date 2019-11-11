cmake_minimum_required(VERSION 3.5)
project(common)
find_package(catkin REQUIRED)

catkin_package()

install(PROGRAMS 
    bin/DRCF/DRCF32 bin/DRCF/DRCF64 bin/DRCF/run_drcf.sh
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY
    imp bin/DRCF/config 
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)