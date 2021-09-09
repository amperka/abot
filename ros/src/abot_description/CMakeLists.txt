cmake_minimum_required(VERSION 3.0.2)
project(abot_description)

find_package(catkin REQUIRED COMPONENTS
  rviz
  tf
  urdf
  xacro
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
