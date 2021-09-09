#include <ros/ros.h>
#include "vc_set_navigation_goal.h"

int main(int argc, char **argv) {
	ros::init(argc, argv, "vc_set_navigation_goal");
	VCSetNavigationGoal set_navigation_goal;
	ROS_INFO("Voice command node 'Set Navigation Goal': Start.");
	ros::spin();
	return 0;
}
