
#include "encoder_wiring_pi.h"
#include <std_msgs/Float64.h>
#include <chrono>

typedef boost::chrono::steady_clock time_source;

class EncodersPair {
public:
    EncodersPair(double update_rate);

private:
    ros::NodeHandle _node;

    ros::Publisher _left_wheel_angle_pub;
    ros::Publisher _right_wheel_angle_pub;
    ros::Publisher _left_wheel_velocity_pub;
    ros::Publisher _right_wheel_velocity_pub;

    ros::Timer _encoders_timer;

    std_msgs::Float64 _left_wheel_angle_msg;
    std_msgs::Float64 _right_wheel_angle_msg;
    std_msgs::Float64 _left_wheel_velocity_msg;
    std_msgs::Float64 _right_wheel_velocity_msg;

    EncoderWiringPi _encoder_left;
    EncoderWiringPi _encoder_right;

    double _left_wheel_angle;
    double _right_wheel_angle;
    double _left_wheel_velocity;
    double _right_wheel_velocity;
    double _left_wheel_position;
    double _right_wheel_position;

    time_source::time_point _last_time;

    void encodersCallback(const ros::TimerEvent& event);
};

EncodersPair::EncodersPair(double update_rate) :
    _encoder_left(ENCODER_1_PIN_A, ENCODER_1_PIN_B, &EncoderWiringPiISR::encoderISR1, &EncoderWiringPiISR::encoder_position_1),
    _encoder_right(ENCODER_2_PIN_A, ENCODER_2_PIN_B, &EncoderWiringPiISR::encoderISR2, &EncoderWiringPiISR::encoder_position_2) {
    _left_wheel_angle_pub = _node.advertise<std_msgs::Float64>("/abot/left_wheel/angle", 1);
    _right_wheel_angle_pub = _node.advertise<std_msgs::Float64>("/abot/right_wheel/angle", 1);
    _left_wheel_velocity_pub = _node.advertise<std_msgs::Float64>("/abot/left_wheel/current_velocity", 1);
    _right_wheel_velocity_pub = _node.advertise<std_msgs::Float64>("/abot/right_wheel/current_velocity", 1);
   
    _encoders_timer = _node.createTimer(ros::Duration(update_rate), &EncodersPair::encodersCallback, this);
}

void EncodersPair::encodersCallback(const ros::TimerEvent& event) {
    time_source::time_point this_time = time_source::now();
    boost::chrono::duration<double> elapsed_duration = this_time - _last_time;
    ros::Duration elapsed(elapsed_duration.count());
    _last_time = this_time;

    _left_wheel_angle = -1 * _encoder_left.getAngle();
    _right_wheel_angle = 1 * _encoder_right.getAngle();

    _left_wheel_angle_msg.data = _left_wheel_angle;
    _right_wheel_angle_msg.data = _right_wheel_angle;

    _left_wheel_angle_pub.publish(_left_wheel_angle_msg);
    _right_wheel_angle_pub.publish(_right_wheel_angle_msg);

    double delta_left_wheel = _left_wheel_angle - _left_wheel_position;
    double delta_right_wheel = _right_wheel_angle - _right_wheel_position;

    _left_wheel_position += delta_left_wheel;
    _left_wheel_velocity = delta_left_wheel / elapsed.toSec();

    _right_wheel_position += delta_right_wheel;
    _right_wheel_velocity = delta_right_wheel / elapsed.toSec();
 
    _left_wheel_velocity_msg.data = _left_wheel_velocity;
    _right_wheel_velocity_msg.data = _right_wheel_velocity;

    _left_wheel_velocity_pub.publish(_left_wheel_velocity_msg);
    _right_wheel_velocity_pub.publish(_right_wheel_velocity_msg);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "encoders");
    EncodersPair encoders_pair(0.01);
    ros::spin();
    return 0;
}