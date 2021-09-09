cmake_minimum_required(VERSION 3.0.2)
project(abot_navigation)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  roscpp
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(simple_navigation_goal src/simple_navigation_goal.cpp)
target_link_libraries(simple_navigation_goal ${catkin_LIBRARIES})