#include <GpioExpanderPi.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <string>
#include <vector>
#include <wiringPi.h>

constexpr uint8_t EXPANDER_GREEN_LED_PIN = 0;
constexpr uint8_t EXPANDER_RED_LED_PIN = 1;

const std::vector<std::string> VOICE_COMMANDS = {
	"включи зеленый светодиод",
	"выключи зеленый светодиод",
	"включи красный светодиод",
	"выключи красный светодиод",

	"включи все светодиоды",
	"выключи все светодиоды",

	"установи яркость зеленого светодиода ноль",
	"установи яркость зеленого светодиода десять",
	"установи яркость зеленого светодиода двадцать",
	"установи яркость зеленого светодиода тридцать",
	"установи яркость зеленого светодиода сорок",
	"установи яркость зеленого светодиода пятьдесят",
	"установи яркость зеленого светодиода шестьдесят",
	"установи яркость зеленого светодиода семьдесят",
	"установи яркость зеленого светодиода восемьдесят",
	"установи яркость зеленого светодиода девяносто",
	"установи яркость зеленого светодиода сто",

	"установи яркость красного светодиода ноль",
	"установи яркость красного светодиода десять",
	"установи яркость красного светодиода двадцать",
	"установи яркость красного светодиода тридцать",
	"установи яркость красного светодиода сорок",
	"установи яркость красного светодиода пятьдесят",
	"установи яркость красного светодиода шестьдесят",
	"установи яркость красного светодиода семьдесят",
	"установи яркость красного светодиода восемьдесят",
	"установи яркость красного светодиода девяносто",
	"установи яркость красного светодиода сто"
};

class TestCommandExecutor {
public:
	TestCommandExecutor();

private:
	ros::NodeHandle _node;
	ros::Subscriber _grammar_sub;

	GpioExpanderPi _expander;

	void executeCommand(uint8_t command_number);
	void grammarCallback(const std_msgs::String::ConstPtr& msg);
};

TestCommandExecutor::TestCommandExecutor() {
	_grammar_sub = _node.subscribe("/abot/stt/grammar_data", 1, &TestCommandExecutor::grammarCallback, this);

	if (!_expander.begin())
		throw std::runtime_error("Test Command Executor node: Expander launch error!");

	_expander.pinMode(EXPANDER_GREEN_LED_PIN, GPIO_PIN_OUTPUT);
	_expander.pinMode(EXPANDER_RED_LED_PIN, GPIO_PIN_OUTPUT);
}

void TestCommandExecutor::grammarCallback(const std_msgs::String::ConstPtr& msg) {
	std::string grammar_string = msg->data.c_str();
	uint8_t total_commands = VOICE_COMMANDS.size();

	for (uint8_t i = 0; i < total_commands; i++)
		if (grammar_string == VOICE_COMMANDS[i])
			executeCommand(i);
}

void TestCommandExecutor::executeCommand(uint8_t command_number) {
	if (command_number == 0)
		_expander.digitalWrite(EXPANDER_GREEN_LED_PIN, HIGH);
	else if (command_number == 1)
		_expander.digitalWrite(EXPANDER_GREEN_LED_PIN, LOW);
	else if (command_number == 2)
		_expander.digitalWrite(EXPANDER_RED_LED_PIN, HIGH);
	else if (command_number == 3)
		_expander.digitalWrite(EXPANDER_RED_LED_PIN, LOW);
	else if (command_number == 4) {
		_expander.digitalWrite(EXPANDER_GREEN_LED_PIN, HIGH);
		_expander.digitalWrite(EXPANDER_RED_LED_PIN, HIGH);
	} else if (command_number == 5) {
		_expander.digitalWrite(EXPANDER_GREEN_LED_PIN, LOW);
		_expander.digitalWrite(EXPANDER_RED_LED_PIN, LOW);
	} else if (command_number >= 6 && command_number < 17) {
		uint8_t command_in_order = command_number - 6;
		uint8_t percent = command_in_order * 10;
		float pwm = 255.0 * percent / 100;
		_expander.analogWrite(EXPANDER_GREEN_LED_PIN, (uint8_t)pwm);
	} else if (command_number >= 17 && command_number < 28) {
		uint8_t command_in_order = command_number - 17;
		uint8_t percent = command_in_order * 10;
		float pwm = 255.0 * percent / 100;
		_expander.analogWrite(EXPANDER_RED_LED_PIN, (uint8_t)pwm);
	}
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "test_command_executor");
	TestCommandExecutor testCommandExecutor;
	ROS_INFO("Test Command Executor node: Start");
	ros::spin();
	return 0;
}
