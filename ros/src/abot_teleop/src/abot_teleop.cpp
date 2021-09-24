#include "abot_teleop.h"

int main(int argc, char** argv) {
	ros::init(argc, argv, "abot_teleop");
	ros::NodeHandle private_node("~");
	AbotTeleop abotTeleop(private_node);
	ros::spin();
}