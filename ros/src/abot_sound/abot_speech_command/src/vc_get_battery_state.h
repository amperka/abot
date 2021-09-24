#ifndef VC_GET_BATTERY_STATE_H_
#define VC_GET_BATTERY_STATE_H_

#include <GpioExpanderPi.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <string>
#include <vector>

const std::vector<std::string> VOICE_COMMANDS_GET_BATTERY_STATE = {
	"заряд батареи",
	"заряд аккумулятора",
	"заряд батареи подробно",
	"заряд аккумулятора подробно"
};

constexpr float V_BATTERY_MAX = 4.2 * 2;
constexpr float V_BATTERY_MIN = 2.75 * 2;
constexpr float V_BATTERY_DIF = V_BATTERY_MAX - V_BATTERY_MIN;
constexpr float V_REF = 3.29;
constexpr float R_DIVIDER = 0.3897307;
constexpr uint8_t GPIO_EXPANDER_DIVIDER_PIN = 7;

class VCGetBatteryState {
public:
	VCGetBatteryState(GpioExpanderPi* expander);

private:
	ros::NodeHandle _node;
	ros::Subscriber _stt_sub;
	ros::Publisher _tts_pub;

	GpioExpanderPi* _expander;
	float _battery_voltage;
	int _battery_percentage;

	void getVoltage();
	std::string makePercentString();
	std::string makeVoltageString();
	void grammarCallback(const std_msgs::String::ConstPtr& msg);
};

VCGetBatteryState::VCGetBatteryState(GpioExpanderPi* expander) {
	_stt_sub = _node.subscribe("/abot/stt/grammar_data", 1, &VCGetBatteryState::grammarCallback, this);
	_tts_pub = _node.advertise<std_msgs::String>("/abot/tts/text_to_say", 1);
	_expander = expander;
}

void VCGetBatteryState::grammarCallback(const std_msgs::String::ConstPtr& msg) {
	std::string grammar_string = msg->data.c_str();
	if (grammar_string == VOICE_COMMANDS_GET_BATTERY_STATE[0] || grammar_string == VOICE_COMMANDS_GET_BATTERY_STATE[1]) {
		getVoltage();
		std_msgs::String tts_string_msg;
		tts_string_msg.data = makePercentString();
		_tts_pub.publish(tts_string_msg);
	} else if (grammar_string == VOICE_COMMANDS_GET_BATTERY_STATE[2] || grammar_string == VOICE_COMMANDS_GET_BATTERY_STATE[3]) {
		getVoltage();
		std_msgs::String tts_string_msg;
		tts_string_msg.data = makePercentString() + makeVoltageString();
		_tts_pub.publish(tts_string_msg);
	}
}

void VCGetBatteryState::getVoltage() {
	uint16_t analog_value = _expander->analogRead(GPIO_EXPANDER_DIVIDER_PIN);
	// printf("%d\n",analog_value);

	float input_voltage = V_REF / 4095.0 * analog_value;
	_battery_voltage = input_voltage / R_DIVIDER;

	if (_battery_voltage < V_BATTERY_MIN)
		_battery_voltage = V_BATTERY_MIN;
	if (_battery_voltage > V_BATTERY_MAX)
		_battery_voltage = V_BATTERY_MAX;

	_battery_percentage = (_battery_voltage - V_BATTERY_MIN) / V_BATTERY_DIF * 100;
}

std::string VCGetBatteryState::makePercentString() {
	int percentage_fisrt_digit = _battery_percentage % 10;

	std::string percentage_word_string;
	if (percentage_fisrt_digit == 1)
		percentage_word_string = " процент";
	else if (percentage_fisrt_digit == 2 || percentage_fisrt_digit == 3 || percentage_fisrt_digit == 4)
		percentage_word_string = " процента";
	else
		percentage_word_string = " процентов";

	std::string percentage_string = "Заряд " + std::to_string(_battery_percentage) + percentage_word_string;
	return percentage_string;
}

std::string VCGetBatteryState::makeVoltageString() {
	int int_part = _battery_voltage;
	int fractal_part = _battery_voltage * 100 - int_part * 100;

	std::string voltage_string = " Напряжение " + std::to_string(int_part) + " точка " + std::to_string(fractal_part) + " вольт";
	return voltage_string;
}

#endif // VC_GET_BATTERY_STATE_H_