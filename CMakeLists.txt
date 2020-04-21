cmake_minimum_required(VERSION 2.8.3)
project(project)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf
)


catkin_package(
)


include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(converter src/sub.cpp)
add_dependencies(converter ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(tfNode src/tfNode.cpp)
add_dependencies(tfNode ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(converter
  ${catkin_LIBRARIES}
)

target_link_libraries(tfNode
  ${catkin_LIBRARIES}
)