cmake_minimum_required(VERSION 2.8.3)
project(motor_test)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES motor_test
  CATKIN_DEPENDS roscpp std_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(motor_node src/motor_node.cpp)
add_dependencies(motor_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(motor_node ${catkin_LIBRARIES} pigpiod_if2)
