#ifndef ICARUSINTERFACE_WHEEL_H
#define ICARUSINTERFACE_WHEEL_H

#include <string>
#include "rclcpp/rclcpp.hpp"


class Wheel
{
    public:

    std::string name = "";
    int enc = 0;
    double cmd = 0; //[rad/s]
    double pos = 0; //[m]
    double vel = 0; //[rad/s]
    double eff = 0; //[pwm]
    double velSetPt = 0; //[rad/s]
    double rads_per_count = 0;
    double desired_speed = 0; //[rad/s]
    double pos_prev = 0; //[m]
    double curr_pwm = 0;
    double error = 0;
    double oldError = 0;
    double integralError = 0;
    double oldIntegralError = 0;
    double proportionalGain = 2.8;
    double integralGain = .01;
    double derivativeGain = .002;
    double time_difference = .067; //[s]



    Wheel() = default;

    Wheel(const std::string &wheel_name, int counts_per_rev);
    
    void setup(const std::string &wheel_name, int counts_per_rev);

    double calcEncAngle(int enc);

    double calculatePID(double desiredValue, double currentValue);



};


#endif // ICARUSINTERFACE_WHEEL_H
