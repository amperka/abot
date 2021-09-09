cmake_minimum_required(VERSION 3.0.2)
project(abot_speech_command)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	std_msgs
	move_base_msgs
	actionlib
)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

add_executable(test_command_executor src/test/test_command_executor.cpp)
target_link_libraries(test_command_executor ${catkin_LIBRARIES} -lwiringPi -lGpioExpanderPi)

add_executable(vc_get_battery_state src/vc_get_battery_state.cpp)
target_link_libraries(vc_get_battery_state ${catkin_LIBRARIES} -lwiringPi -lGpioExpanderPi)

add_executable(vc_get_system_time src/vc_get_system_time.cpp)
target_link_libraries(vc_get_system_time ${catkin_LIBRARIES})

add_executable(vc_set_navigation_goal src/vc_set_navigation_goal.cpp)
target_link_libraries(vc_set_navigation_goal ${catkin_LIBRARIES})