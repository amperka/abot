#include "vc_get_battery_state.h"
#include <GpioExpanderPi.h>
#include <ros/ros.h>

int main(int argc, char** argv) {
	ros::init(argc, argv, "vc_get_battery_state");

	GpioExpanderPi expander;
	if (!expander.begin()) {
		ROS_ERROR("Voice command node 'Get Battery State': Failed to init I2C communication.");
		return -1;
	}

	VCGetBatteryState get_battery_state(&expander);

	ROS_INFO("Voice command node 'Get Battery State': Start.");
	ros::spin();
	return 0;
}
