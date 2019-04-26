#---- gripper src --------------------------
SET( SRC_SIMPLE_PICK_AND_PLACE
  ${COMMON_SRC_FILES}
  src/gripper/pick_and_place_simple.cpp
)

SET( SRC_SIMPLE_REAL_PICK_AND_PLACE
  ${COMMON_SRC_FILES}
  src/gripper/real_pick_and_place_simple.cpp
)

SET( SRC_SIMPLE_OBJECT_PICK_AND_PLACE
  ${COMMON_SRC_FILES}
  src/gripper/object_pick_and_place_simple.cpp
)

SET( SRC_FILES_SERIAL_EXAMPLE_NODE
  ${COMMON_SRC_FILES}
  src/gripper/serial_example_node.cpp
)

#---- mobile src --------------------------
SET( SRC_FILES_SINGLE_ROBOT_MOBILE
  ${COMMON_SRC_FILES}
  src/mobile/single_robot_mobile.cpp
)
SET( SRC_FILES_MULTI_ROBOT_MOBILE
  ${COMMON_SRC_FILES}
  src/mobile/multi_robot_mobile.cpp
)
SET( SRC_FILES_SINGLE_ROBOT_MOBILE_CIRCLE
  ${COMMON_SRC_FILES}
  src/mobile/single_robot_mobile_circle.cpp
)



#---- demo src -----------------------------
SET( SRC_SIMPLE_DANCE_M1013
  ${COMMON_SRC_FILES}
  src/demo/dance_m1013.cpp
)
SET( SRC_MULTI_M1013x2
  ${COMMON_SRC_FILES}
  src/demo/m1013x2.cpp
)
SET( SRC_MULTI_M1013x2_NO_SYNC
  ${COMMON_SRC_FILES}
  src/demo/m1013x2_no_sync.cpp
)
SET( SRC_MULTI_M1013x2_SYNC
  ${COMMON_SRC_FILES}
  src/demo/m1013x2_sync.cpp
)
SET( SRC_MULTI_M1013x2_SYNC2
  ${COMMON_SRC_FILES}
  src/demo/m1013x2_sync2.cpp
)
SET( SRC_FILES_MULTI_ROBOT
  ${COMMON_SRC_FILES}
  src/demo/multi_robot.cpp
)


#----- Gripper ----------------------------------------------------------------------------------------
add_executable(pick_and_place ${SRC_SIMPLE_PICK_AND_PLACE})
add_dependencies(pick_and_place ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(pick_and_place ${catkin_LIBRARIES})

add_executable(real_pick_and_place ${SRC_SIMPLE_REAL_PICK_AND_PLACE})
add_dependencies(real_pick_and_place ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(real_pick_and_place ${catkin_LIBRARIES})

add_executable(object_pick_and_place ${SRC_SIMPLE_OBJECT_PICK_AND_PLACE})
add_dependencies(object_pick_and_place ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(object_pick_and_place ${catkin_LIBRARIES})

add_executable(serial_example_node ${SRC_FILES_SERIAL_EXAMPLE_NODE})
add_dependencies(serial_example_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(serial_example_node ${catkin_LIBRARIES})

#----- mobile -----------------------------------------------------------------------------------------
add_executable(single_robot_mobile ${SRC_FILES_SINGLE_ROBOT_MOBILE})
add_dependencies(single_robot_mobile ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(single_robot_mobile ${catkin_LIBRARIES})

add_executable(multi_robot_mobile ${SRC_FILES_MULTI_ROBOT_MOBILE})
add_dependencies(multi_robot_mobile ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(multi_robot_mobile ${catkin_LIBRARIES})

add_executable(single_robot_mobile_circle ${SRC_FILES_SINGLE_ROBOT_MOBILE_CIRCLE})
add_dependencies(single_robot_mobile_circle ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(single_robot_mobile_circle ${catkin_LIBRARIES})



#----- demo -------------------------------------------------------------------------------------------
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

add_executable(multi_robot ${SRC_FILES_MULTI_ROBOT})
add_dependencies(multi_robot ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(multi_robot ${catkin_LIBRARIES})

add_executable(dance_m1013 ${SRC_SIMPLE_DANCE_M1013})
add_dependencies(dance_m1013 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(dance_m1013 ${catkin_LIBRARIES})
