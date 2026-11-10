#ifndef ICARUSINTERFACE_MOTOR_CONTROL_H
#define ICARUSINTERFACE_MOTOR_CONTROL_H

#include <iostream>
#include <cstring>
#include "rclcpp/rclcpp.hpp"
#include <pigpiod_if2.h>

#include <unistd.h> 

#include "config_rasp_pi.h"

using namespace std;

class motor_control
{
    public:
        motor_control();
        ~motor_control();

        void motor_config(motor m);
        int start_motors();
        void setMotorMode(const string &mode, motor m);
        void setMotor(const double &power, motor m); 


    private:
        rclcpp::Logger logger_;
};





#endif // ICARUSINTERFACE_MOTOR_CONTROL_H
