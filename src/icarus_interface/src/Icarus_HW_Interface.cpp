#include "icarus_interface/Icarus_HW_Interface.h"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pid_messages/msg/pid.hpp"

//GLOBALS
int pi_;
Wheel fl_wheel_;
Wheel fr_wheel_;
Wheel bl_wheel_;
Wheel br_wheel_;

static rclcpp::Time now_time;

double deltaPositionFL = 0;
double deltaPositionFR = 0;
double deltaPositionBL = 0;
double deltaPositionBR = 0;

// Lazy publisher holders (created on_configure using the HW node's executor context if available,
// otherwise a private standalone node like before). For Jazzy, we avoid creating
// publishers during plugin load (before node creation). We create them on_configure.
static std::shared_ptr<rclcpp::Node> pidTracker;
static rclcpp::Publisher<pid_messages::msg::Pid>::SharedPtr pidTrackingFl;
static rclcpp::Publisher<pid_messages::msg::Pid>::SharedPtr pidTrackingFr;
static rclcpp::Publisher<pid_messages::msg::Pid>::SharedPtr pidTrackingBl;
static rclcpp::Publisher<pid_messages::msg::Pid>::SharedPtr pidTrackingBr;

IcarusInterface::IcarusInterface()
    : logger_(rclcpp::get_logger("IcarusInterface")),
    currentTime_(rclcpp::Clock().now()),
    previousTime_(currentTime_)
{}

IcarusInterface::~IcarusInterface()
{
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn IcarusInterface::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  auto logger = rclcpp::get_logger("IcarusInterface.on_init");
  RCLCPP_INFO(logger, "On init ...");

  auto & info = params.hardware_info;

  //taken from URDF information published by robot_state_publisher
  try {
    cfg_.front_left_wheel_name = info.hardware_parameters.at("front_left_wheel_name");
    cfg_.back_left_wheel_name = info.hardware_parameters.at("back_left_wheel_name");
    cfg_.front_right_wheel_name = info.hardware_parameters.at("front_right_wheel_name");
    cfg_.back_right_wheel_name = info.hardware_parameters.at("back_right_wheel_name");
    cfg_.loop_rate = std::stof(info.hardware_parameters.at("loop_rate"));
    cfg_.enc_counts_per_rev = std::stoi(info.hardware_parameters.at("enc_counts_per_rev"));
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger, "Failed to parse hardware parameters in on_init: %s", e.what());
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  }

  // Set up the wheels
  fl_wheel_.setup(cfg_.front_left_wheel_name, cfg_.enc_counts_per_rev);
  bl_wheel_.setup(cfg_.back_left_wheel_name, cfg_.enc_counts_per_rev);
  fr_wheel_.setup(cfg_.front_right_wheel_name, cfg_.enc_counts_per_rev);
  br_wheel_.setup(cfg_.back_right_wheel_name, cfg_.enc_counts_per_rev);

  RCLCPP_INFO(logger, "Finished on_init");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn IcarusInterface::on_configure(
  const rclcpp_lifecycle::State & previous_state)
{
  (void)previous_state;
  RCLCPP_INFO(logger_, "Configuring...");

  time_ = std::chrono::system_clock::now();

  // Lazily create PID tracker publishers here (avoids creating nodes at plugin load time)
  if (!pidTracker) {
    pidTracker = std::make_shared<rclcpp::Node>("pidTracker");
    pidTrackingFl = pidTracker->create_publisher<pid_messages::msg::Pid>("pidTracker", 10);
    pidTrackingFr = pidTracker->create_publisher<pid_messages::msg::Pid>("pidTracker", 10);
    pidTrackingBl = pidTracker->create_publisher<pid_messages::msg::Pid>("pidTracker", 10);
    pidTrackingBr = pidTracker->create_publisher<pid_messages::msg::Pid>("pidTracker", 10);
  }

  pi_ = pigpio_start(OPTHOST, OPTPORT);
  if (pi_ < 0)
  {
    RCLCPP_INFO(logger_, ("----gpio failed to initialize (motor)"));
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  }

  //setup motor_control
  motor_ctr.start_motors();
  enc_ctr.start_encoders();

  RCLCPP_INFO(logger_, "Finished Configuration");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface::ConstSharedPtr> IcarusInterface::on_export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface::ConstSharedPtr> state_interfaces;

  state_interfaces.push_back(std::make_shared<hardware_interface::StateInterface>(
    fl_wheel_.name, hardware_interface::HW_IF_VELOCITY, &fl_wheel_.vel));
  state_interfaces.push_back(std::make_shared<hardware_interface::StateInterface>(
    fl_wheel_.name, hardware_interface::HW_IF_POSITION, &fl_wheel_.pos));
  state_interfaces.push_back(std::make_shared<hardware_interface::StateInterface>(
    fr_wheel_.name, hardware_interface::HW_IF_VELOCITY, &fr_wheel_.vel));
  state_interfaces.push_back(std::make_shared<hardware_interface::StateInterface>(
    fr_wheel_.name, hardware_interface::HW_IF_POSITION, &fr_wheel_.pos));
  state_interfaces.push_back(std::make_shared<hardware_interface::StateInterface>(
    bl_wheel_.name, hardware_interface::HW_IF_VELOCITY, &bl_wheel_.vel));
  state_interfaces.push_back(std::make_shared<hardware_interface::StateInterface>(
    bl_wheel_.name, hardware_interface::HW_IF_POSITION, &bl_wheel_.pos));
  state_interfaces.push_back(std::make_shared<hardware_interface::StateInterface>(
    br_wheel_.name, hardware_interface::HW_IF_VELOCITY, &br_wheel_.vel));
  state_interfaces.push_back(std::make_shared<hardware_interface::StateInterface>(
    br_wheel_.name, hardware_interface::HW_IF_POSITION, &br_wheel_.pos));

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface::SharedPtr> IcarusInterface::on_export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface::SharedPtr> command_interfaces;

  command_interfaces.push_back(std::make_shared<hardware_interface::CommandInterface>(
    fl_wheel_.name, hardware_interface::HW_IF_VELOCITY, &fl_wheel_.cmd));
  command_interfaces.push_back(std::make_shared<hardware_interface::CommandInterface>(
    fr_wheel_.name, hardware_interface::HW_IF_VELOCITY, &fr_wheel_.cmd));
  command_interfaces.push_back(std::make_shared<hardware_interface::CommandInterface>(
    bl_wheel_.name, hardware_interface::HW_IF_VELOCITY, &bl_wheel_.cmd));
  command_interfaces.push_back(std::make_shared<hardware_interface::CommandInterface>(
    br_wheel_.name, hardware_interface::HW_IF_VELOCITY, &br_wheel_.cmd));

  return command_interfaces;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn IcarusInterface::on_activate(
  const rclcpp_lifecycle::State & previous_state)
{
  (void)previous_state;
  RCLCPP_INFO(logger_, "Starting Controller...");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn IcarusInterface::on_deactivate(
  const rclcpp_lifecycle::State & previous_state)
{
  (void)previous_state;
  RCLCPP_INFO(logger_, "Stopping Controller...");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type IcarusInterface::read(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  (void)time;
  (void)period;
  // In the original code read() did nothing; position tracking happens in write().
  // Keep it as a no-op (encoders are polled in write()).
  return return_type::OK;
}

hardware_interface::return_type IcarusInterface::write(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  (void)time;
  currentTime_ = time;
  double dt = period.seconds();
  if (dt <= 0.0) {
    dt = 0.067; // fallback from wheel.h default if period is zero on first tick
  }
  fl_wheel_.time_difference = dt;
  fr_wheel_.time_difference = dt;
  bl_wheel_.time_difference = dt;
  br_wheel_.time_difference = dt;

  //track position for each wheel
  //current position is already tracked by encoder control file
  fl_wheel_.pos = fl_wheel_.calcEncAngle(fl_wheel_.enc);
  deltaPositionFL = fl_wheel_.pos - fl_wheel_.pos_prev;
  fl_wheel_.pos_prev = fl_wheel_.pos;

  fr_wheel_.pos = fr_wheel_.calcEncAngle(fr_wheel_.enc);
  deltaPositionFR = fr_wheel_.pos - fr_wheel_.pos_prev;
  fr_wheel_.pos_prev = fr_wheel_.pos;

  bl_wheel_.pos = bl_wheel_.calcEncAngle(bl_wheel_.enc);
  deltaPositionBL = bl_wheel_.pos - bl_wheel_.pos_prev;
  bl_wheel_.pos_prev = bl_wheel_.pos;

  br_wheel_.pos = br_wheel_.calcEncAngle(br_wheel_.enc);
  deltaPositionBR = br_wheel_.pos - br_wheel_.pos_prev;
  br_wheel_.pos_prev = br_wheel_.pos;

  //track velocity for each wheel
  fl_wheel_.vel = deltaPositionFL/ fl_wheel_.time_difference;
  fr_wheel_.vel = deltaPositionFR/ fr_wheel_.time_difference;
  bl_wheel_.vel = deltaPositionBL/ bl_wheel_.time_difference;
  br_wheel_.vel = deltaPositionBR/ br_wheel_.time_difference;

  //constrain our desired speed to be between two values
  fl_wheel_.desired_speed = std::clamp( fl_wheel_.cmd , -10.0 , 10.0);
  bl_wheel_.desired_speed = std::clamp( bl_wheel_.cmd , -10.0 , 10.0);
  fr_wheel_.desired_speed = std::clamp( fr_wheel_.cmd , -10.0 , 10.0);
  br_wheel_.desired_speed = std::clamp( br_wheel_.cmd , -10.0 , 10.0);

  //calcualte the effort needed to reach desired speed
  fl_wheel_.eff = fl_wheel_.calculatePID(fl_wheel_.desired_speed,fl_wheel_.vel);
  bl_wheel_.eff = bl_wheel_.calculatePID(bl_wheel_.desired_speed,bl_wheel_.vel);
  fr_wheel_.eff = fr_wheel_.calculatePID(fr_wheel_.desired_speed,fr_wheel_.vel);
  br_wheel_.eff = br_wheel_.calculatePID(br_wheel_.desired_speed,br_wheel_.vel);

  if (pidTrackingFl) {
    //create topics to track and tune PID response
    auto msgFl = pid_messages::msg::Pid();
    msgFl.header.stamp = currentTime_;
    msgFl.name = {"fl_wheel"};
    msgFl.desired_value = {fl_wheel_.desired_speed};
    msgFl.measured_value = {fl_wheel_.vel};
    msgFl.effort = {fl_wheel_.eff};

    auto msgFr = pid_messages::msg::Pid();
    msgFr.header.stamp = currentTime_;
    msgFr.name = {"fr_wheel"};
    msgFr.desired_value = {fr_wheel_.desired_speed};
    msgFr.measured_value = {fr_wheel_.vel};
    msgFr.effort = {fr_wheel_.eff};

    auto msgBl = pid_messages::msg::Pid();
    msgBl.header.stamp = currentTime_;
    msgBl.name = {"bl_wheel"};
    msgBl.desired_value = {bl_wheel_.desired_speed};
    msgBl.measured_value = {bl_wheel_.vel};
    msgBl.effort = {bl_wheel_.eff};

    auto msgBr = pid_messages::msg::Pid();
    msgBr.header.stamp = currentTime_;
    msgBr.name = {"br_wheel"};
    msgBr.desired_value = {br_wheel_.desired_speed};
    msgBr.measured_value = {br_wheel_.vel};
    msgBr.effort = {br_wheel_.eff};

    //publish topic messages
    pidTrackingFl->publish(msgFl);
    pidTrackingFr->publish(msgFr);
    pidTrackingBl->publish(msgBl);
    pidTrackingBr->publish(msgBr);
  }

  //apply PID calculations to motor pwm
  fl_wheel_.curr_pwm += fl_wheel_.eff;
  motor_ctr.setMotor(fl_wheel_.curr_pwm, MOTOR_FL);

  bl_wheel_.curr_pwm += bl_wheel_.eff;
  motor_ctr.setMotor((bl_wheel_.curr_pwm), MOTOR_BL);

  fr_wheel_.curr_pwm += fr_wheel_.eff;
  motor_ctr.setMotor((fr_wheel_.curr_pwm), MOTOR_FR);

  br_wheel_.curr_pwm += br_wheel_.eff;
  motor_ctr.setMotor((br_wheel_.curr_pwm), MOTOR_BR);

  return return_type::OK;
}



#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  IcarusInterface,
  hardware_interface::SystemInterface
)
