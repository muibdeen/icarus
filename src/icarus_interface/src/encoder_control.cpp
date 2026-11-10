#include "icarus_interface/encoder_control.h"
#include "icarus_interface/motor_control.h"

//Global varaibles
void callbackFL(int currentPosition)
{
    fl_wheel_.enc = currentPosition;
}

void callbackFR(int currentPosition)
{   
    fr_wheel_.enc = currentPosition;
}

void callbackBL(int currentPosition)
{
    bl_wheel_.enc = currentPosition;
}

void callbackBR(int currentPosition)
{
    br_wheel_.enc = currentPosition;
}

encoder_control::encoder_control()
    : logger_(rclcpp::get_logger("encoder_control"))
{}


int encoder_control::start_encoders()
{
    RCLCPP_INFO(logger_, ("----Starting encoders" ));
    
    if (pi_ < 0) 
    {
        RCLCPP_INFO(logger_, ("----gpio failed to initialize (encoder)" ));
        return 1;
    }
    else
        RCLCPP_INFO(logger_, ("----PiGPIO version::  %i" ), get_pigpio_version(pi_));

    REDencFL = RED(pi_, ENCODER_FL.OPTGPIOA, ENCODER_FL.OPTGPIOB, optMode, callbackFL);
    REDencFR = RED(pi_, ENCODER_FR.OPTGPIOA, ENCODER_FR.OPTGPIOB, optMode, callbackFR);
    REDencBL = RED(pi_, ENCODER_BL.OPTGPIOA, ENCODER_BL.OPTGPIOB, optMode, callbackBL);
    REDencBR = RED(pi_, ENCODER_BR.OPTGPIOA, ENCODER_BR.OPTGPIOB, optMode, callbackBR);
    RED_set_glitch_filter(REDencFL, ENCODER_FL.OPTGLITCH);
    RED_set_glitch_filter(REDencFR, ENCODER_FR.OPTGLITCH);
    RED_set_glitch_filter(REDencBL, ENCODER_BL.OPTGLITCH);
    RED_set_glitch_filter(REDencBR, ENCODER_BR.OPTGLITCH);

    return 0;
}

encoder_control::~encoder_control()
{
    RCLCPP_INFO(logger_, ("----Cleaning up encoder GPIO"));

    if (pi_ >= 0)
    {
        RED_cancel(REDencFL);
        RED_cancel(REDencFR);
        RED_cancel(REDencBL);
        RED_cancel(REDencBR);
        RCLCPP_INFO(logger_, ("----encoder GPIO Stopped"));
    }

}
