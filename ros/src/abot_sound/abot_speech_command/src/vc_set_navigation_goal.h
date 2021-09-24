#ifndef VC_SET_NAVIGATION_GOAL_H_
#define VC_SET_NAVIGATION_GOAL_H_

#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <string>
#include <vector>

const std::vector<std::string> VOICE_COMMANDS_SET_NAVIGATION_GOAL = {
	"едь домой",
	"едь к михаилу",
	"едь к виктору",
	"едь к антону",
	"разворот",
	"поворот влево",
	"поворот вправо",
	"движение вперёд на пол метра",
	"движение вперёд на метр",
	"движение назад на пол метра",
	"движение назад на метр",
};

constexpr double POSE_HOME[7] = { 0.042, -0.013, 0.018, 0.000, 0.000, 0.031, 1.000 };
constexpr double POSE_MIKHAIL[7] = { -1.651, 6.007, 0.018, 0.000, 0.000, 0.675, 0.738 };
constexpr double POSE_VICTOR[7] = { -0.978, -4.205, 0.018, 0.000, 0.000, 0.023, 1.000 };
constexpr double POSE_ANTON[7] = { 12.615, -0.322, 0.018, 0.000, 0.000, 0.728, 0.685 };

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class VCSetNavigationGoal {
public:
	VCSetNavigationGoal();

private:
	ros::NodeHandle _node;
	ros::Subscriber _stt_sub;
	ros::Publisher _tts_pub;

	MoveBaseClient _ac { "/move_base", true };

	void sendGoalMsg(const std::string frame_id, const double parameters[7]);

	void grammarCallback(const std_msgs::String::ConstPtr& msg);
};

VCSetNavigationGoal::VCSetNavigationGoal() {
	_stt_sub = _node.subscribe("/abot/stt/grammar_data", 1, &VCSetNavigationGoal::grammarCallback, this);
	_tts_pub = _node.advertise<std_msgs::String>("/abot/tts/text_to_say", 1);
	while (!_ac.waitForServer(ros::Duration(5.0))) {
		ROS_INFO("Voice command node 'Set Navigation Goal': Waiting for the move_base action server to come up");
	}
}

void VCSetNavigationGoal::sendGoalMsg(const std::string frame_id, const double parameters[7]) {
	move_base_msgs::MoveBaseGoal goal;
	goal.target_pose.header.frame_id = frame_id;
	goal.target_pose.header.stamp = ros::Time::now();
	goal.target_pose.pose.position.x = parameters[0];
	goal.target_pose.pose.position.y = parameters[1];
	goal.target_pose.pose.position.z = parameters[2];
	goal.target_pose.pose.orientation.x = parameters[3];
	goal.target_pose.pose.orientation.y = parameters[4];
	goal.target_pose.pose.orientation.z = parameters[5];
	goal.target_pose.pose.orientation.w = parameters[6];
	_ac.sendGoal(goal);
}

void VCSetNavigationGoal::grammarCallback(const std_msgs::String::ConstPtr& text_msg) {
	std::string grammar_string = text_msg->data.c_str();
	std_msgs::String answer_msg;
	if (grammar_string == VOICE_COMMANDS_SET_NAVIGATION_GOAL[0]) {
		answer_msg.data = "Еду домой!";
		_tts_pub.publish(answer_msg);
		sendGoalMsg("map", POSE_HOME);
	} else if (grammar_string == VOICE_COMMANDS_SET_NAVIGATION_GOAL[1]) {
		answer_msg.data = "Еду к Михаилу!";
		_tts_pub.publish(answer_msg);
		sendGoalMsg("map", POSE_MIKHAIL);
	} else if (grammar_string == VOICE_COMMANDS_SET_NAVIGATION_GOAL[2]) {
		answer_msg.data = "Еду к Виктору!";
		_tts_pub.publish(answer_msg);
		sendGoalMsg("map", POSE_VICTOR);
	} else if (grammar_string == VOICE_COMMANDS_SET_NAVIGATION_GOAL[3]) {
		answer_msg.data = "Еду к Антону!";
		_tts_pub.publish(answer_msg);
		sendGoalMsg("map", POSE_ANTON);
	} else if (grammar_string == VOICE_COMMANDS_SET_NAVIGATION_GOAL[4]) {
		answer_msg.data = "Выполняю разворот на месте!";
		_tts_pub.publish(answer_msg);
		double params[7] = { 0, 0, 0, 0, 0, 1, 0 };
		sendGoalMsg("base_link", params);
	} else if (grammar_string == VOICE_COMMANDS_SET_NAVIGATION_GOAL[5]) {
		answer_msg.data = "Выполняю поворот влево!";
		_tts_pub.publish(answer_msg);
		double params[7] = { 0, 0, 0, 0, 0, 0.707, 0.707 };
		sendGoalMsg("base_link", params);
	} else if (grammar_string == VOICE_COMMANDS_SET_NAVIGATION_GOAL[6]) {
		answer_msg.data = "Выполняю поворот вправо!";
		_tts_pub.publish(answer_msg);
		double params[7] = { 0, 0, 0, 0, 0, -0.707, 0.707 };
		sendGoalMsg("base_link", params);
	} else if (grammar_string == VOICE_COMMANDS_SET_NAVIGATION_GOAL[7]) {
		answer_msg.data = "Выполняю!";
		_tts_pub.publish(answer_msg);
		double params[7] = { 0.5, 0, 0, 0, 0, 0, 1 };
		sendGoalMsg("base_link", params);
	} else if (grammar_string == VOICE_COMMANDS_SET_NAVIGATION_GOAL[8]) {
		answer_msg.data = "Выполняю!";
		_tts_pub.publish(answer_msg);
		double params[7] = { 1.0, 0, 0, 0, 0, 0, 1 };
		sendGoalMsg("base_link", params);
	} else if (grammar_string == VOICE_COMMANDS_SET_NAVIGATION_GOAL[9]) {
		answer_msg.data = "Выполняю!";
		_tts_pub.publish(answer_msg);
		double params[7] = { -0.5, 0, 0, 0, 0, 0, 1 };
		sendGoalMsg("base_link", params);
	} else if (grammar_string == VOICE_COMMANDS_SET_NAVIGATION_GOAL[10]) {
		answer_msg.data = "Выполняю!";
		_tts_pub.publish(answer_msg);
		double params[7] = { -1.0, 0, 0, 0, 0, 0, 1 };
		sendGoalMsg("base_link", params);
	}
}

#endif // SET_NAVIGATION_GOAL_H_
