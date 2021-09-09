cmake_minimum_required(VERSION 3.0.2)
project(abot_teleop)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  joy
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(abot_teleop src/abot_teleop.cpp)
target_link_libraries(abot_teleop ${catkin_LIBRARIES})
