#include "icarus_interface/wheel.h"

#include <cmath>


Wheel::Wheel(const std::string &wheel_name, int counts_per_rev)
{
  setup(wheel_name, counts_per_rev);
}


void Wheel::setup(const std::string &wheel_name, int counts_per_rev)
{
  name = wheel_name;
  rads_per_count = (2*M_PI)/counts_per_rev;
}

double Wheel::calcEncAngle(int enc)
{
  return enc * rads_per_count;
}

double Wheel::calculatePID(double desiredValue, double currentValue)
{
  error = desiredValue - currentValue;
  integralError = oldIntegralError + (time_difference*(error + oldError)/2);
  double pidValue = proportionalGain * error + integralGain * integralError + (derivativeGain * (error - oldError) / time_difference);

  return pidValue;
}