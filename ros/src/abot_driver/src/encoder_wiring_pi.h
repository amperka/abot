#ifndef ENCODER_WIRING_PI_H_
#define ENCODER_WIRING_PI_H_

#include <ros/ros.h>
#include <wiringPi.h>

constexpr uint8_t ENCODER_1_PIN_A = 17; // Wiring pi 0 = BCM 17
constexpr uint8_t ENCODER_1_PIN_B = 27; // Wiring pi 2 = BCM 27
constexpr uint8_t ENCODER_2_PIN_A = 24; // Wiring pi 5 = BCM 24
constexpr uint8_t ENCODER_2_PIN_B = 25; // Wiring pi 6 = BCM 25

constexpr uint16_t PULSES_PER_REVOLUTION = 1920;

namespace EncoderWiringPiISR {

volatile long encoder_position_1;
volatile long encoder_position_2;
volatile uint8_t encoder_state_1;
volatile uint8_t encoder_state_2;

void encoderISR(const int pin_A, const int pin_B, volatile long& encoder_position, volatile uint8_t& encoder_state) {
	uint8_t val_A = digitalRead(pin_A);
	uint8_t val_B = digitalRead(pin_B);
	uint8_t s = encoder_state & 3;
	if (val_A)
		s |= 4;
	if (val_B)
		s |= 8;
	encoder_state = (s >> 2);
	if (s == 1 || s == 7 || s == 8 || s == 14)
		encoder_position++;
	else if (s == 2 || s == 4 || s == 11 || s == 13)
		encoder_position--;
	else if (s == 3 || s == 12)
		encoder_position += 2;
	else if (s == 6 || s == 9)
		encoder_position -= 2;
}

void encoderISR1(void) {
	encoderISR(ENCODER_1_PIN_A, ENCODER_1_PIN_B, encoder_position_1, encoder_state_1);
}

void encoderISR2(void) {
	encoderISR(ENCODER_2_PIN_A, ENCODER_2_PIN_B, encoder_position_2, encoder_state_2);
}
}

class EncoderWiringPi {
public:
	EncoderWiringPi(const int& pin_A, const int& pin_B, void (*isrFunction)(void), volatile long* encoder_position);
	double getAngle();

private:
	int _pin_A;
	int _pin_B;
	volatile long* _encoder_position;
	double _initial_angle;
	double ticks2Angle(long position);
};

EncoderWiringPi::EncoderWiringPi(const int& pin_A, const int& pin_B, void (*isrFunction)(void), volatile long* encoder_position) {
	_encoder_position = encoder_position;

	if (wiringPiSetupSys() < 0) {
		throw std::runtime_error("Encoder wiringPi error: GPIO setup error");
	}

	ROS_INFO("Encoder wiringPi: GPIO setup");
	_pin_A = pin_A;
	_pin_B = pin_B;
	pinMode(_pin_A, INPUT);
	pinMode(_pin_B, INPUT);
	pullUpDnControl(_pin_A, PUD_UP);
	pullUpDnControl(_pin_B, PUD_UP);

	if (wiringPiISR(_pin_A, INT_EDGE_BOTH, isrFunction) < 0) {
		throw std::runtime_error("Encoder wiringPi error: ISR pinA error");
	}

	if (wiringPiISR(_pin_B, INT_EDGE_BOTH, isrFunction) < 0) {
		throw std::runtime_error("Encoder wiringPi error: ISR pinB error");
	}

	_initial_angle = ticks2Angle(*_encoder_position);
	ROS_INFO("Encoder wiringPi: ISR setup");
}

double EncoderWiringPi::getAngle() {
	double current_angle = ticks2Angle(*_encoder_position);
	return current_angle - _initial_angle;
}

double EncoderWiringPi::ticks2Angle(long position) {
	return position * ((double)2 * M_PI / PULSES_PER_REVOLUTION / 2);
}

#endif // ENCODER_WIRING_PI_H_