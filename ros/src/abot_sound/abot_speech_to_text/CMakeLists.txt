cmake_minimum_required(VERSION 3.0.2)
project(abot_speech_to_text)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	rospy
	std_msgs
)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

catkin_install_python(PROGRAMS
	scripts/asr_control.py
	scripts/kws_control.py
	DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(FILES
	launch/abot_speech_to_text.launch
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY
	launch
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})