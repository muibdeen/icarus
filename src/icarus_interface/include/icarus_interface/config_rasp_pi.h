

#ifndef ICARUSINTERFACE_CONFIG_RASP_PI_H
#define ICARUSINTERFACE_CONFIG_RASP_PI_H    
#include "wheel.h"
#include "control_toolbox/pid.hpp"

using control_toolbox::Pid;
//===============================================================
//  STRUCTS
//===============================================================
struct encoder{
    const uint8_t OPTGPIOA; //
    const uint8_t OPTGPIOB; //
    const uint8_t OPTGLITCH; //Constant
};

struct motor{
    const uint16_t PWM_MAX;
    const uint8_t ENA; //enableMotor Pin
    const uint8_t IN1; //forwardPin
    const uint8_t IN2; //backwardPin
    const uint8_t FREQ; //
    const uint16_t RANGE; //
};

//===============================================================
//  CONSTANTS
//===============================================================
//encoder PID value
const double PROPORTIONAL_GAIN = 0;
const double INTERGRAL_GAIN = 0;
const double DERIVATIVE_GAIN = 0;
const double I_MIN = 0.3;
const double I_MAX = -0.3;
const bool ANTIWINDUP = true;

// motor = {PWM_MAX, ENA, IN1, IN2, FREQ, RANGE}
const uint16_t PWM_MAX = 255;
const uint8_t FREQ = 200;
const uint16_t RANGE = 255;

const motor MOTOR_FL = {PWM_MAX, 21, 16, 20, FREQ, RANGE};
const motor MOTOR_FR = {PWM_MAX, 13, 5, 6, FREQ, RANGE};
const motor MOTOR_BL = {PWM_MAX, 2, 4, 3, FREQ, RANGE};
const motor MOTOR_BR = {PWM_MAX, 25, 23, 24, FREQ, RANGE};

// encoder = {OptA, OptB, Glitch}
const uint8_t OPTGLITCH_DEFAULT = 250;

const encoder ENCODER_FL = {26, 19, OPTGLITCH_DEFAULT};
const encoder ENCODER_FR = {1, 12, OPTGLITCH_DEFAULT};
const encoder ENCODER_BL = {27, 17, OPTGLITCH_DEFAULT};
const encoder ENCODER_BR = {8, 7, OPTGLITCH_DEFAULT};


//===============================================================
//  GLOBAL VARIABLES
//===============================================================
extern int pi_;

extern Wheel fl_wheel_;
extern Wheel fr_wheel_;
extern Wheel bl_wheel_;
extern Wheel br_wheel_;
extern int effortFL;

extern control_toolbox::Pid pidFL;
extern control_toolbox::Pid pidBL;
extern control_toolbox::Pid pidFR;
extern control_toolbox::Pid pidBR;

#endif // ICARUSINTERFACE_CONFIG_RASP_PI_H
