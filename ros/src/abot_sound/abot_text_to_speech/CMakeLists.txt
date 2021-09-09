cmake_minimum_required(VERSION 3.0.2)
project(abot_text_to_speech)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	rospy
	std_msgs
)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

catkin_install_python(PROGRAMS
	scripts/aws_polly_tts.py
	scripts/festvox_tts.py
	scripts/rhvoice_tts.py
	DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(FILES
	launch/abot_abot_text_to_speech.launch
	launch/aws_polly_tts.launch
	launch/festvox_tts.launch
	launch/rhvoice_tts.launch
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY
	launch
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

