#ifndef ICARUSINTERFACE_ENCODER_CONTROL_H
#define ICARUSINTERFACE_ENCODER_CONTROL_H

#include <iostream>
#include <cstring>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/clock.hpp"
#include "control_toolbox/pid.hpp"
#include <pigpiod_if2.h>

#include "rotary_encoder.h"
#include "config_rasp_pi.h"
#include "wheel.h"
#include "sensor_msgs/msg/joint_state.hpp"

#include<unistd.h> 

using namespace std;
using control_toolbox::Pid;

class encoder_control
{
    public:
        encoder_control();
        ~encoder_control();

        int start_encoders();
        int read_encoders();

    private:

        rclcpp::Logger logger_;

        RED_t *REDencFL;
        RED_t *REDencFR;
        RED_t *REDencBL;
        RED_t *REDencBR;

        int optMode = RED_MODE_DETENT;
};
#endif // ICARUSINTERFACE_ENCODER_CONTROL_H