// Copyright 2020 PAL Robotics S.L.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/*
 * Author: Bence Magyar, Enrique Fernández, Manuel Meraz
 */

#include <memory>
#include <queue>
#include <string>
#include <utility>
#include <vector>

#include "mech_drive_controller/mech_drive_controller.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/logging.hpp"
#include "tf2/LinearMath/Quaternion.h"

namespace
{
constexpr auto DEFAULT_COMMAND_TOPIC = "~/cmd_vel";
constexpr auto DEFAULT_COMMAND_UNSTAMPED_TOPIC = "~/cmd_vel_unstamped";
constexpr auto DEFAULT_COMMAND_OUT_TOPIC = "~/cmd_vel_out";
constexpr auto DEFAULT_ODOMETRY_TOPIC = "/odom";
constexpr auto DEFAULT_TRANSFORM_TOPIC = "/tf";
}  // namespace

namespace mech_drive_controller
{
using namespace std::chrono_literals;
using controller_interface::interface_configuration_type;
using controller_interface::InterfaceConfiguration;
using hardware_interface::HW_IF_POSITION;
using hardware_interface::HW_IF_VELOCITY;
using lifecycle_msgs::msg::State;

MechDriveController::MechDriveController() : controller_interface::ControllerInterface() {}

controller_interface::CallbackReturn MechDriveController::on_init()
{
  try
  {
    // with the lifecycle node being initialized, we can declare parameters that are called out in the yaml
    auto_declare<std::vector<std::string>>("front_left_wheel_name", std::vector<std::string>());
    auto_declare<std::vector<std::string>>("front_right_wheel_name", std::vector<std::string>());
    auto_declare<std::vector<std::string>>("back_left_wheel_name", std::vector<std::string>());
    auto_declare<std::vector<std::string>>("back_right_wheel_name", std::vector<std::string>());

    auto_declare<double>("wheel_separation_width", wheel_params_.separation_width);
    auto_declare<double>("wheel_separation_length", wheel_params_.separation_length);

    auto_declare<int>("wheels_per_side", static_cast<int>(wheel_params_.wheels_per_side));
    auto_declare<double>("wheel_radius", wheel_params_.radius);

    auto_declare<std::string>("odom_frame_id", odom_params_.odom_frame_id);
    auto_declare<std::string>("base_frame_id", odom_params_.base_frame_id);
    auto_declare<std::vector<double>>("pose_covariance_diagonal", std::vector<double>());
    auto_declare<std::vector<double>>("twist_covariance_diagonal", std::vector<double>());
    auto_declare<bool>("open_loop", odom_params_.open_loop);
    auto_declare<bool>("enable_odom_tf", odom_params_.enable_odom_tf);

    auto_declare<double>("cmd_vel_timeout", cmd_vel_timeout_.count() / 1000.0);
    auto_declare<bool>("publish_limited_velocity", publish_limited_velocity_);
    auto_declare<int>("velocity_rolling_window_size", 50);
    auto_declare<bool>("use_stamped_vel", use_stamped_vel_);

    auto_declare<double>("publish_rate", publish_rate_);
  }
  catch (const std::exception & e)
  {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

InterfaceConfiguration MechDriveController::command_interface_configuration() const
{
  //declare each joint to be claimed by the velocity command interface
  std::vector<std::string> conf_names;
  //joint name inherits the value within _wheel name
  for (const auto & joint_name : front_left_wheel_name)
  {
    conf_names.push_back(joint_name + "/" + HW_IF_VELOCITY);
  }
  for (const auto & joint_name : back_right_wheel_name)
  {
    conf_names.push_back(joint_name + "/" + HW_IF_VELOCITY);
  }
  for (const auto & joint_name : back_left_wheel_name)
  {
    conf_names.push_back(joint_name + "/" + HW_IF_VELOCITY);
  }
  for (const auto & joint_name : front_right_wheel_name)
  {
    conf_names.push_back(joint_name + "/" + HW_IF_VELOCITY);
  }
  return {interface_configuration_type::INDIVIDUAL, conf_names};
}

InterfaceConfiguration MechDriveController::state_interface_configuration() const
{
  //declare each joint to be claimed by the position state interface
  std::vector<std::string> conf_names;

  for (const auto & joint_name : back_left_wheel_name)
  {
    conf_names.push_back(joint_name + "/" + HW_IF_POSITION);
  }
  for (const auto & joint_name : front_right_wheel_name)
  {
    conf_names.push_back(joint_name + "/" + HW_IF_POSITION);
  }
    for (const auto & joint_name : front_left_wheel_name)
  {
    conf_names.push_back(joint_name + "/" + HW_IF_POSITION);
  }
  for (const auto & joint_name : back_right_wheel_name)
  {
    conf_names.push_back(joint_name + "/" + HW_IF_POSITION);
  }
  return {interface_configuration_type::INDIVIDUAL, conf_names};
}

controller_interface::return_type MechDriveController::update(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  (void)period;
  auto node = get_node();
  auto logger = node->get_logger();
  if (get_lifecycle_state().id() == State::PRIMARY_STATE_INACTIVE)
  {
    if (!is_halted)
    {
      halt();
      is_halted = true;
    }
    return controller_interface::return_type::OK;
  }

  const auto current_time = time;

  std::shared_ptr<Twist> last_msg;
  received_velocity_msg_ptr_.get(last_msg);

  if (last_msg == nullptr)
  {
    RCLCPP_WARN(logger, "Velocity message received was a nullptr.");
    return controller_interface::return_type::ERROR;
  }

  const auto dt = current_time - last_msg->header.stamp;
  // Brake if cmd_vel has timeout, override the stored command
  if (dt > cmd_vel_timeout_)
  {
    last_msg->twist.linear.x = 0.0;
    last_msg->twist.angular.z = 0.0;
    last_msg->twist.linear.y = 0.0;
  }

  // command may be limited further by SpeedLimit,
  // without affecting the stored twist command
  Twist command = *last_msg;
  std::vector<double> twist_cmd({0,0});
  double & x = command.twist.linear.x;
  double & z = command.twist.angular.z;
  double & y = command.twist.linear.y;  

  // Apply wheel params from my_controller.yaml:
  const auto wheels = wheel_params_;
  const double wheel_separation_length =wheels.separation_length;
  const double wheel_separation_width =wheels.separation_width;
  const double left_wheel_radius = wheels.radius;
  const double right_wheel_radius = wheels.radius;

  if (odom_params_.open_loop)
  {
    //in the case of no encoders:
    odometry_.updateOpenLoop(x, y, z, current_time);
  }
  else
  {
    //track position to be used for runga-kutta integration
    const double front_left_position = registered_front_left_wheel_handle_[0].position.get().get_value();
    const double front_right_position = registered_front_right_wheel_handle_[0].position.get().get_value();
    const double back_left_position = registered_back_left_wheel_handle_[0].position.get().get_value();
    const double back_right_position = registered_back_right_wheel_handle_[0].position.get().get_value();      

    if (std::isnan(front_left_position) || std::isnan(front_right_position) || std::isnan(back_left_position) || std::isnan(back_right_position))
    {
      RCLCPP_ERROR(
        logger, "Either the left or right wheel positions are invalid");
      return controller_interface::return_type::ERROR;
    }
    odometry_.update(front_left_position, front_right_position, back_left_position, back_right_position, current_time);
  }

  tf2::Quaternion orientation;
  //rotate robot as heading is updated within odometry
  orientation.setRPY(0.0, 0.0, odometry_.getHeading());

  if (previous_publish_timestamp_ + publish_period_ < current_time)
  {
    previous_publish_timestamp_ += publish_period_;

    //messages published to odom topic
    //the odom topic has an unmoving fixed frame. the messages sent are the distance between the robot and the odom frame
    if (realtime_odometry_publisher_->trylock())
    {
      auto & odometry_message = realtime_odometry_publisher_->msg_;
      odometry_message.header.stamp = current_time;
      odometry_message.pose.pose.position.x = odometry_.getX();
      odometry_message.pose.pose.position.y = odometry_.getY();
      odometry_message.pose.pose.orientation.x = orientation.x();
      odometry_message.pose.pose.orientation.y = orientation.y();
      odometry_message.pose.pose.orientation.z = orientation.z();
      odometry_message.pose.pose.orientation.w = orientation.w();
      odometry_message.twist.twist.linear.x = odometry_.getLinear_x();
      odometry_message.twist.twist.linear.y = odometry_.getLinear_y();
      odometry_message.twist.twist.angular.z = odometry_.getAngular();
      realtime_odometry_publisher_->unlockAndPublish();
    }

    // this is used if we want odometry to publish directly to TF 
    if (odom_params_.enable_odom_tf && realtime_odometry_transform_publisher_->trylock())
    {
      auto & transform = realtime_odometry_transform_publisher_->msg_.transforms.front();
      transform.header.stamp = current_time;
      transform.transform.translation.x = odometry_.getX();
      transform.transform.translation.y = odometry_.getY();
      transform.transform.rotation.x = orientation.x();
      transform.transform.rotation.y = orientation.y();
      transform.transform.rotation.z = orientation.z();
      transform.transform.rotation.w = orientation.w();
      realtime_odometry_transform_publisher_->unlockAndPublish();
    }
  }

  const auto update_dt = current_time - previous_update_timestamp_;
  previous_update_timestamp_ = current_time;
  (void)update_dt;

  previous_commands_.pop();
  previous_commands_.emplace(command);

  //Publish limited velocity
  if (publish_limited_velocity_ && realtime_limited_velocity_publisher_->trylock())
  {
    auto & limited_velocity_command = realtime_limited_velocity_publisher_->msg_;
    limited_velocity_command.header.stamp = current_time;
    limited_velocity_command.twist = command.twist;
    realtime_limited_velocity_publisher_->unlockAndPublish();
  }

  // Compute wheels velocities:
  const double velocity_front_left =
    (x - y - z * (wheel_separation_length + wheel_separation_width) / 2) / left_wheel_radius;
  const double velocity_front_right =
    (x + y + z * (wheel_separation_length + wheel_separation_width) / 2) / right_wheel_radius;
    const double velocity_back_left =
    (x + y - z * (wheel_separation_length + wheel_separation_width) / 2) / left_wheel_radius;
  const double velocity_back_right =
    (x - y + z * (wheel_separation_length + wheel_separation_width) / 2) / right_wheel_radius;

  // Set wheels velocities:
    registered_front_left_wheel_handle_[0].velocity.get().set_value(velocity_front_left);
    registered_front_right_wheel_handle_[0].velocity.get().set_value(velocity_front_right);
    registered_back_left_wheel_handle_[0].velocity.get().set_value(velocity_back_left);
    registered_back_right_wheel_handle_[0].velocity.get().set_value(velocity_back_right);
  

  return controller_interface::return_type::OK;
}

CallbackReturn MechDriveController::on_configure(const rclcpp_lifecycle::State & previous_state)
{
  (void)previous_state;
  auto node = get_node();
  auto logger = node->get_logger();

  // update parameters
  back_left_wheel_name = node->get_parameter("back_left_wheel_name").as_string_array();
  back_right_wheel_name = node->get_parameter("back_right_wheel_name").as_string_array();
  front_left_wheel_name = node->get_parameter("front_left_wheel_name").as_string_array();
  front_right_wheel_name = node->get_parameter("front_right_wheel_name").as_string_array();

  //confirm that amount of wheels on both sides are equal
  if (back_left_wheel_name.size() + front_left_wheel_name.size()  != front_right_wheel_name.size() + back_right_wheel_name.size())
  {
    RCLCPP_ERROR(
      logger, "The number of left wheels [%zu] and the number of right wheels [%zu] are different",
      back_left_wheel_name.size() + front_left_wheel_name.size(), front_right_wheel_name.size() + back_right_wheel_name.size());
    return CallbackReturn::ERROR;
  }

  if (front_left_wheel_name.empty() || back_left_wheel_name.empty())
  {
    RCLCPP_ERROR(logger, "Wheel names parameters are empty!");
    return CallbackReturn::ERROR;
  }
  //wheel parameters are taken from the my controller config file within this node package
  wheel_params_.separation_length = node->get_parameter("wheel_separation_length").as_double();
  wheel_params_.separation_width = node->get_parameter("wheel_separation_width").as_double();
  wheel_params_.wheels_per_side = static_cast<size_t>(node->get_parameter("wheels_per_side").as_int());
  wheel_params_.radius = node->get_parameter("wheel_radius").as_double();

  //apply wheel params from my_controller.yaml
  const auto wheels = wheel_params_;
  const double wheel_separation_length =wheels.separation_length;
  const double wheel_separation_width =wheels.separation_width;
  const double left_wheel_radius = wheels.radius;
  const double right_wheel_radius = wheels.radius;

  odometry_.setWheelParams(wheel_separation_length, wheel_separation_width, left_wheel_radius, right_wheel_radius);
  odometry_.setVelocityRollingWindowSize(node->get_parameter("velocity_rolling_window_size").as_int());

  odom_params_.odom_frame_id = node->get_parameter("odom_frame_id").as_string();
  odom_params_.base_frame_id = node->get_parameter("base_frame_id").as_string();

  auto pose_diagonal = node->get_parameter("pose_covariance_diagonal").as_double_array();
  std::copy(
    pose_diagonal.begin(), pose_diagonal.end(), odom_params_.pose_covariance_diagonal.begin());

  auto twist_diagonal = node->get_parameter("twist_covariance_diagonal").as_double_array();
  std::copy(
    twist_diagonal.begin(), twist_diagonal.end(), odom_params_.twist_covariance_diagonal.begin());

  odom_params_.open_loop = node->get_parameter("open_loop").as_bool();
  odom_params_.enable_odom_tf = node->get_parameter("enable_odom_tf").as_bool();

  cmd_vel_timeout_ = std::chrono::milliseconds{
    static_cast<int>(node->get_parameter("cmd_vel_timeout").as_double() * 1000.0)};
  publish_limited_velocity_ = node->get_parameter("publish_limited_velocity").as_bool();
  use_stamped_vel_ = node->get_parameter("use_stamped_vel").as_bool();


  if (!reset())
  {
    return CallbackReturn::ERROR;
  }

  //set the amount of expected wheels per side for the robot
  wheel_params_.wheels_per_side = node->get_parameter("wheels_per_side").as_int();

  if (publish_limited_velocity_)
  {
    limited_velocity_publisher_ = node->create_publisher<Twist>(DEFAULT_COMMAND_OUT_TOPIC, rclcpp::SystemDefaultsQoS());
    realtime_limited_velocity_publisher_ = std::make_shared<realtime_tools::RealtimePublisher<Twist>>(limited_velocity_publisher_);
  }

  const Twist empty_twist;
  received_velocity_msg_ptr_.set(std::make_shared<Twist>(empty_twist));

  // Fill last two commands with default constructed commands
  previous_commands_.emplace(empty_twist);
  previous_commands_.emplace(empty_twist);

  // initialize command subscriber
  if (use_stamped_vel_)
  {
    velocity_command_subscriber_ = node->create_subscription<Twist>(
      DEFAULT_COMMAND_TOPIC, rclcpp::SystemDefaultsQoS(),
      [this](const std::shared_ptr<Twist> msg) -> void {
        auto node = get_node();
        if (!subscriber_is_active_)
        {
          RCLCPP_WARN(node->get_logger(), "Can't accept new commands. subscriber is inactive");
          return;
        }
        if ((msg->header.stamp.sec == 0) && (msg->header.stamp.nanosec == 0))
        {
          RCLCPP_WARN_ONCE(
            node->get_logger(),
            "Received TwistStamped with zero timestamp, setting it to current "
            "time, this message will only be shown once");
          msg->header.stamp = node->get_clock()->now();
        }
        received_velocity_msg_ptr_.set(std::move(msg));
      });
  }
  else
  {
    velocity_command_unstamped_subscriber_ = node->create_subscription<geometry_msgs::msg::Twist>(
      DEFAULT_COMMAND_UNSTAMPED_TOPIC, rclcpp::SystemDefaultsQoS(),
      [this](const std::shared_ptr<geometry_msgs::msg::Twist> msg) -> void {
        auto node = get_node();
        (void)node;
        if (!subscriber_is_active_)
        {
          RCLCPP_WARN(node->get_logger(), "Can't accept new commands. subscriber is inactive");
          return;
        }

        // Write fake header in the stored stamped command
        std::shared_ptr<Twist> twist_stamped;
        received_velocity_msg_ptr_.get(twist_stamped);
        twist_stamped->twist = *msg;
        twist_stamped->header.stamp = node->get_clock()->now();
      });
  }

  // initialize odometry publisher and messasge
  odometry_publisher_ = node->create_publisher<nav_msgs::msg::Odometry>(
    DEFAULT_ODOMETRY_TOPIC, rclcpp::SystemDefaultsQoS());
  realtime_odometry_publisher_ =
    std::make_shared<realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>>(
      odometry_publisher_);

  auto & odometry_message = realtime_odometry_publisher_->msg_;
  odometry_message.header.frame_id = odom_params_.odom_frame_id;
  odometry_message.child_frame_id = odom_params_.base_frame_id;

  // limit the publication on the topics /odom and /tf
  publish_rate_ = node->get_parameter("publish_rate").as_double();
  publish_period_ = rclcpp::Duration::from_seconds(1.0 / publish_rate_);
  previous_publish_timestamp_ = node->get_clock()->now();

  // initialize odom values zeros
  odometry_message.twist =
    geometry_msgs::msg::TwistWithCovariance(rosidl_runtime_cpp::MessageInitialization::ALL);

  constexpr size_t NUM_DIMENSIONS = 6;
  for (size_t index = 0; index < 6; ++index)
  {
    // 0, 7, 14, 21, 28, 35
    const size_t diagonal_index = NUM_DIMENSIONS * index + index;
    odometry_message.pose.covariance[diagonal_index] = odom_params_.pose_covariance_diagonal[index];
    odometry_message.twist.covariance[diagonal_index] =
      odom_params_.twist_covariance_diagonal[index];
  }

  // initialize transform publisher and message
  odometry_transform_publisher_ = node->create_publisher<tf2_msgs::msg::TFMessage>(
    DEFAULT_TRANSFORM_TOPIC, rclcpp::SystemDefaultsQoS());
  realtime_odometry_transform_publisher_ =
    std::make_shared<realtime_tools::RealtimePublisher<tf2_msgs::msg::TFMessage>>(
      odometry_transform_publisher_);

  // keeping track of odom and base_link transforms only
  auto & odometry_transform_message = realtime_odometry_transform_publisher_->msg_;
  odometry_transform_message.transforms.resize(1);
  odometry_transform_message.transforms.front().header.frame_id = odom_params_.odom_frame_id;
  odometry_transform_message.transforms.front().child_frame_id = odom_params_.base_frame_id;

  previous_update_timestamp_ = node->get_clock()->now();
  return CallbackReturn::SUCCESS;
}

//set the wheel joints upon node activation
CallbackReturn MechDriveController::on_activate(const rclcpp_lifecycle::State & previous_state)
{
  (void)previous_state;
  auto node = get_node();
  const auto front_left_result = configure_side("front_left", front_left_wheel_name, registered_front_left_wheel_handle_);
  const auto front_right_result = configure_side("front_right", front_right_wheel_name, registered_front_right_wheel_handle_);
  const auto back_left_result = configure_side("back_left", back_left_wheel_name, registered_back_left_wheel_handle_);
  const auto back_right_result = configure_side("back_right", back_right_wheel_name, registered_back_right_wheel_handle_);

  if (front_left_result == CallbackReturn::ERROR || front_right_result == CallbackReturn::ERROR || back_left_result == CallbackReturn::ERROR || back_right_result == CallbackReturn::ERROR)
  {
    return CallbackReturn::ERROR;
  }

  if (registered_front_left_wheel_handle_.empty() || registered_front_right_wheel_handle_.empty() || registered_back_left_wheel_handle_.empty() || registered_back_right_wheel_handle_.empty())
  {
    RCLCPP_ERROR(node->get_logger(), "Either left wheel interfaces, right wheel interfaces are non existent");
    return CallbackReturn::ERROR;
  }

  is_halted = false;
  subscriber_is_active_ = true;

  RCLCPP_DEBUG(node->get_logger(), "Subscriber and publisher are now active.");
  return CallbackReturn::SUCCESS;
}

CallbackReturn MechDriveController::on_deactivate(const rclcpp_lifecycle::State & previous_state)
{
  (void)previous_state;
  subscriber_is_active_ = false;
  return CallbackReturn::SUCCESS;
}

CallbackReturn MechDriveController::on_cleanup(const rclcpp_lifecycle::State & previous_state)
{
  (void)previous_state;
  if (!reset())
  {
    return CallbackReturn::ERROR;
  }

  received_velocity_msg_ptr_.set(std::make_shared<Twist>());
  return CallbackReturn::SUCCESS;
}

CallbackReturn MechDriveController::on_error(const rclcpp_lifecycle::State & previous_state)
{
  (void)previous_state;
  if (!reset())
  {
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

bool MechDriveController::reset()
{
  odometry_.resetOdometry();

  // release the old queue
  std::queue<Twist> empty;
  std::swap(previous_commands_, empty);

  registered_front_left_wheel_handle_.clear();
  registered_front_right_wheel_handle_.clear();
  registered_back_left_wheel_handle_.clear();
  registered_back_right_wheel_handle_.clear();

  subscriber_is_active_ = false;
  velocity_command_subscriber_.reset();
  velocity_command_unstamped_subscriber_.reset();

  received_velocity_msg_ptr_.set(nullptr);
  is_halted = false;
  return true;
}

CallbackReturn MechDriveController::on_shutdown(const rclcpp_lifecycle::State & previous_state)
{
  (void)previous_state;
  return CallbackReturn::SUCCESS;
}

void MechDriveController::halt()
{
  const auto halt_wheels = [](auto & wheel_handles) {
    for (const auto & wheel_handle : wheel_handles)
    {
      wheel_handle.velocity.get().set_value(0.0);
    }
  };

  halt_wheels(registered_front_left_wheel_handle_);
  halt_wheels(registered_front_right_wheel_handle_);
  halt_wheels(registered_back_left_wheel_handle_);
  halt_wheels(registered_back_right_wheel_handle_);
}

CallbackReturn MechDriveController::configure_side(const std::string & side, const std::vector<std::string> & wheel_name,std::vector<WheelHandle> & registered_handles)
{
  auto node = get_node();
  auto logger = node->get_logger();

  if (wheel_name.empty())
  {
    RCLCPP_ERROR(logger, "No '%s' wheel names specified", side.c_str());
    return CallbackReturn::ERROR;
  }

  // register handles
  registered_handles.reserve(wheel_name.size());
  for (const auto & wheel_name : wheel_name)
  {
    const auto state_handle = std::find_if(
      state_interfaces_.cbegin(), state_interfaces_.cend(), [&wheel_name](const auto & interface) {
        return interface.get_prefix_name() == wheel_name &&
               interface.get_interface_name() == HW_IF_POSITION;
      });

    if (state_handle == state_interfaces_.cend())
    {
      RCLCPP_ERROR(logger, "Unable to obtain joint state handle for %s", wheel_name.c_str());
      return CallbackReturn::ERROR;
    }

    const auto command_handle = std::find_if(
      command_interfaces_.begin(), command_interfaces_.end(),
      [&wheel_name](const auto & interface) {
        return interface.get_prefix_name() == wheel_name &&
               interface.get_interface_name() == HW_IF_VELOCITY;
      });

    if (command_handle == command_interfaces_.end())
    {
      RCLCPP_ERROR(logger, "Unable to obtain joint command handle for %s", wheel_name.c_str());
      return CallbackReturn::ERROR;
    }

    registered_handles.emplace_back(
      WheelHandle{std::ref(*state_handle), std::ref(*command_handle)});
  }

  return CallbackReturn::SUCCESS;
}
}  // namespace mech_drive_controller


#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(mech_drive_controller::MechDriveController, controller_interface::ControllerInterface)
