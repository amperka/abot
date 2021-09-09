#include <ros/ros.h>
#include "vc_get_system_time.h"

int main(int argc, char **argv) {
	ros::init(argc, argv, "vc_get_system_time");
	VCGetSystemTime get_system_time;
	ROS_INFO("Voice command node 'Get System Time': Start.");
	ros::spin();
	return 0;
}
