#ifndef ICARUSINTERFACE_REAL_ROBOT_H
#define ICARUSINTERFACE_REAL_ROBOT_H

#include <cstring>
#include <cmath>
#include <pigpiod_if2.h>
#include "rclcpp/rclcpp.hpp"

#include "hardware_interface/hardware_component_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_component_interface_params.hpp"

#include "config.h"
#include "wheel.h"
#include "motor_control.h"
#include "encoder_control.h"
#include "config_rasp_pi.h"


using hardware_interface::return_type;
using namespace std;

class IcarusInterface : public hardware_interface::SystemInterface
{
 public:
  IcarusInterface();
  ~IcarusInterface() override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface::ConstSharedPtr> on_export_state_interfaces()
    override;

  std::vector<hardware_interface::CommandInterface::SharedPtr> on_export_command_interfaces()
    override;

  return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;


 private:

  Config cfg_;

  motor_control motor_ctr;
  encoder_control enc_ctr;

  rclcpp::Logger logger_;

  rclcpp::Time currentTime_;
  rclcpp::Time previousTime_;
  std::chrono::time_point<std::chrono::system_clock> time_;

  std::vector<double> debug;

  char* OPTHOST = NULL;
  char* OPTPORT = NULL;

  // Keep pointers to exported interfaces so read/write can use framework-managed values if needed
  std::vector<hardware_interface::StateInterface::ConstSharedPtr> state_interfaces_;
  std::vector<hardware_interface::CommandInterface::SharedPtr> command_interfaces_;

};


#endif // ICARUSINTERFACE_REAL_ROBOT_H
