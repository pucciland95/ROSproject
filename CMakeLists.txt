cmake_minimum_required(VERSION 2.8.3)
project(project)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf
  nav_msgs
  message_filters
  message_generation
  dynamic_reconfigure
)

generate_dynamic_reconfigure_options(
  cfg/parameters.cfg
  
)

add_service_files(DIRECTORY srv
  FILES
  distance.srv
)

add_message_files(DIRECTORY msg
    FILES
    customMsg.msg
)


generate_messages(
    DEPENDENCIES
    std_msgs
    nav_msgs
)

catkin_package( CATKIN_DEPENDS
  message_filters
  nav_msgs
  message_runtime
)


include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(converter src/sub.cpp)
add_dependencies(converter ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(tfNode src/tfNode.cpp)
add_dependencies(tfNode ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(distComp src/distComp.cpp)
add_dependencies(distComp ${${PROJECT_NAME}_EXPORTED_TARGETS})
add_dependencies(distComp ${catkin_EXPORTED_TARGETS})
add_dependencies(distComp project_generate_messages_cpp)
target_link_libraries(distComp ${catkin_LIBRARIES})

add_executable(distanceSrv src/distanceSrv.cpp)
target_link_libraries(distanceSrv ${catkin_LIBRARIES})
add_dependencies(distanceSrv ${catkin_EXPORTED_TARGETS})

add_executable(clientDist src/client.cpp)
target_link_libraries(clientDist ${catkin_LIBRARIES})
add_dependencies(clientDist ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(converter
  ${catkin_LIBRARIES}
)

target_link_libraries(tfNode
  ${catkin_LIBRARIES}
)
