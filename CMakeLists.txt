cmake_minimum_required(VERSION 2.8.3)
project(offb)





find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)
add_service_files(
  FILES 
  cmdsrv.srv
  send_position.srv
  )


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES offb
  CATKIN_DEPENDS roscpp std_msgs message_runtime
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)


add_executable(offb_node src/offb_node.cpp)
#add_dependencies(offb_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(offb_node ${catkin_LIBRARIES})

add_executable(offb_node2 src/offb_node2.cpp) 
#add_dependencies(offb_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(offb_node2 ${catkin_LIBRARIES})

add_executable(control_node src/control_node.cpp)
#add_dependencies(offb_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(control_node ${catkin_LIBRARIES})