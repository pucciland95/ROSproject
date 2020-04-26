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
)

add_message_files(DIRECTORY msg
    FILES
    customMsg.msg
)


 generate_messages(
    DEPENDENCIES
    std_msgs
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

## Specify libraries to link a library or executable target against
target_link_libraries(converter
  ${catkin_LIBRARIES}
)

target_link_libraries(tfNode
  ${catkin_LIBRARIES}
)
