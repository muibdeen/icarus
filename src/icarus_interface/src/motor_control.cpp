#include "icarus_interface/motor_control.h"

motor_control::motor_control()
    : logger_(rclcpp::get_logger("motor_control"))
{}

int motor_control::start_motors(){
    RCLCPP_INFO(logger_, ("----Starting motors" ));

    motor_config(MOTOR_FL);
    motor_config(MOTOR_BL);
    motor_config(MOTOR_FR);
    motor_config(MOTOR_BR);

    return 0;
}

void motor_control::motor_config(motor m){
    set_mode(pi_,m.ENA, PI_OUTPUT);
    set_mode(pi_,m.IN1, PI_OUTPUT);
    set_mode(pi_,m.IN2, PI_OUTPUT);
    gpio_write(pi_,m.IN1, PI_LOW);
    gpio_write(pi_,m.IN2, PI_LOW);
    set_PWM_frequency(pi_,m.ENA, m.FREQ);
    set_PWM_range(pi_,m.ENA, m.RANGE);
    set_PWM_dutycycle(pi_,m.ENA, 0);
    RCLCPP_INFO(logger_, ("ENA: %i, IN1: %i, IN2: %i, FREQ: %i, RANGE: %i"), m.ENA,m.IN1,m.IN2,m.FREQ,m.RANGE);
}

void motor_control::setMotorMode(const string &mode, motor m) {
    //RCLCPP_INFO(logger_, ("set motor mode to %s \n"), mode.c_str());
    if(mode == "forward") {
        gpio_write(pi_, m.IN1, PI_LOW);
        gpio_write(pi_, m.IN2, PI_HIGH);
    } else if(mode == "reverse") {
        gpio_write(pi_, m.IN1, PI_HIGH);
        gpio_write(pi_, m.IN2, PI_LOW);
    } else {
        gpio_write(pi_, m.IN1, PI_LOW);
        gpio_write(pi_, m.IN2, PI_LOW);
    }
}

void motor_control::setMotor(const double &power, motor m) {
    uint16_t pwm;
    if(power > 20) {
        setMotorMode("forward", m);
        pwm = (int)(power);
    } else if(power < -20) {
        setMotorMode("reverse", m);
        pwm = -(int)(power);
    } else {
        setMotorMode("stop", m);
        pwm = 0;
    }

    if(pwm > m.PWM_MAX) {
        pwm = m.PWM_MAX;
    }

    set_PWM_dutycycle(pi_,m.ENA, pwm);
}

motor_control::~motor_control()
{
    RCLCPP_INFO(logger_, ("----Cleaning up motor GPIO"));

    if (pi_ >= 0)
    {   
        setMotor(0, MOTOR_FL);
        setMotor(0, MOTOR_BL);
        setMotor(0, MOTOR_FR);
        setMotor(0, MOTOR_BR);
        RCLCPP_INFO(logger_, ("----motor GPIO Stopped"));
    }
    if (pi_ >= 0)
    {   
        RCLCPP_INFO(logger_, ("----Stopping Pi"));
        pigpio_stop(pi_);
    }

}

