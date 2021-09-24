#ifndef VC_GET_SYSTEM_TIME_H_
#define VC_GET_SYSTEM_TIME_H_

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <string>
#include <time.h>
#include <vector>

const std::vector<std::string> VOICE_COMMANDS_GET_SYSTEM_TIME = {
	"который час",
	"сколько времени",
	"текущая дата",
	"сегодняшнее число"
};

const std::vector<std::string> DAY_STRINGS = {
	"первое", "второе", "третье", "четвёртое",
	"пятое", "шестое", "седьмое", "восьмое",
	"девятое", "десятое", "одиннадцатое", "двенадцатое",
	"тринадцатое", "четырнадцатое", "пятнадцатое", "шестнадцатое",
	"семнадцатое", "восемнадцатое", "девятнадцатое", "двадцатое",
	"двадцать первое", "двадцать второе", "двадцать третье",
	"двадацать четвёртое", "двадцать пятое", "двадцать шестое",
	"двадцать седьмое", "двадцать восьмое", "двадцать девятое",
	"тридцатое", "тридцать первое"
};

const std::vector<std::string> MONTH_STRINGS = {
	"января", "февраля", "марта", "апреля",
	"мая", "июня", "июля", "августа",
	"сентября", "октября", "ноября", "декабря"
};

const std::vector<std::string> DAY_OF_A_WEEK_STRINGS = {
	"понедельник", "вторник", "среда", "четверг",
	"пятница", "суббота", "воскресенье"
};

class VCGetSystemTime {
public:
	VCGetSystemTime();

private:
	ros::NodeHandle _node;
	ros::Subscriber _stt_sub;
	ros::Publisher _tts_pub;

	time_t _rawtime;
	struct tm* _timeinfo;

	void getTimeInfo();
	std::string makeTimeString();
	std::string makeDateString();
	void grammarCallback(const std_msgs::String::ConstPtr& msg);
};

VCGetSystemTime::VCGetSystemTime() {
	_stt_sub = _node.subscribe("/abot/stt/grammar_data", 1, &VCGetSystemTime::grammarCallback, this);
	_tts_pub = _node.advertise<std_msgs::String>("/abot/tts/text_to_say", 1);
}

void VCGetSystemTime::grammarCallback(const std_msgs::String::ConstPtr& msg) {
	std::string grammar_string = msg->data.c_str();
	if (grammar_string == VOICE_COMMANDS_GET_SYSTEM_TIME[0] || grammar_string == VOICE_COMMANDS_GET_SYSTEM_TIME[1]) {
		getTimeInfo();
		std_msgs::String tts_string_msg;
		tts_string_msg.data = makeTimeString();
		_tts_pub.publish(tts_string_msg);
	}
	if (grammar_string == VOICE_COMMANDS_GET_SYSTEM_TIME[2] || grammar_string == VOICE_COMMANDS_GET_SYSTEM_TIME[3]) {
		getTimeInfo();
		std_msgs::String tts_string_msg;
		tts_string_msg.data = makeDateString();
		_tts_pub.publish(tts_string_msg);
	}
}

void VCGetSystemTime::getTimeInfo() {
	time(&_rawtime);
	_timeinfo = localtime(&_rawtime);
}

std::string VCGetSystemTime::makeTimeString() {
	int hours = (_timeinfo->tm_hour) % 24;
	int minutes = _timeinfo->tm_min;

	std::string hours_word_string;
	std::string hours_string = std::to_string(hours);

	if (hours == 1 || hours == 21)
		hours_word_string = " час ";
	else if ((hours >= 2 && hours <= 4) || hours == 22 || hours == 23)
		hours_word_string = " часа ";
	else
		hours_word_string = " часов ";

	int minutes_second_digit = minutes / 10;
	int minutes_fisrt_digit = minutes % 10;

	std::string minutes_string;
	if (minutes == 1)
		minutes_string = "одна";
	else if (minutes_fisrt_digit == 1 && minutes_second_digit != 1)
		minutes_string = std::to_string(minutes - 1) + " одна";
	else if (minutes == 2)
		minutes_string = "две";
	else if (minutes_fisrt_digit == 2 && minutes_second_digit != 1)
		minutes_string = std::to_string(minutes - 2) + " две";
	else
		minutes_string = std::to_string(minutes);

	std::string minutes_word_string;
	if (minutes_fisrt_digit == 1)
		minutes_word_string = " минута";
	else if (minutes_fisrt_digit == 2 || minutes_fisrt_digit == 3 || minutes_fisrt_digit == 4)
		minutes_word_string = " минуты";
	else
		minutes_word_string = " минут";

	std::string time_string = "Время " + hours_string + hours_word_string + minutes_string + minutes_word_string;
	return time_string;
}

std::string VCGetSystemTime::makeDateString() {
	int day = _timeinfo->tm_mday;
	int month = _timeinfo->tm_mon;
	int week_day = _timeinfo->tm_wday;

	std::string day_string = DAY_STRINGS[day - 1];
	std::string month_string = MONTH_STRINGS[month];
	std::string week_day_string = DAY_OF_A_WEEK_STRINGS[week_day - 1];

	std::string date_string = " Дата " + day_string + " " + month_string + " " + week_day_string;
	return date_string;
}

#endif // VC_GET_SYSTEM_TIME_H_