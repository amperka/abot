cmake_minimum_required(VERSION 3.0.2)
project(abot_navigation)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  sensor_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

LINK_DIRECTORIES("usr/local/lib")
