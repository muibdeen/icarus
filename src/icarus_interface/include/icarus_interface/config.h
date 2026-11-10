#ifndef ICARUSINTERFACE_CONFIG_H
#define ICARUSINTERFACE_CONFIG_H

#include <string>


struct Config
{
  std::string front_left_wheel_name = "front_left_wheel_name";
  std::string back_left_wheel_name = "back_left_wheel_name";
  std::string front_right_wheel_name = "front_right_wheel_name";
  std::string back_right_wheel_name = "back_right_wheel_name";
  float loop_rate = 30;

  int enc_counts_per_rev = 0;
};


#endif // ICARUSINTERFACE_CONFIG_H