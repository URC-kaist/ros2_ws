#include "mr2_rover_control/twist_to_commands_controller.hpp"

#include <algorithm>
#include <cmath>

#include "pluginlib/class_list_macros.hpp"

namespace mr2_rover_control
{

controller_interface::CallbackReturn TwistToCommandsController::on_init()
{
  auto_declare<std::vector<std::string>>("wheel_joints", {});
  auto_declare<std::vector<std::string>>("steering_joints", {});
  auto_declare<double>("wheel_base", 0.94);
  auto_declare<double>("track_width", 0.65);
  auto_declare<double>("wheel_radius", 0.11);
  auto_declare<double>("max_steer", 0.6);
  auto_declare<double>("twist_timeout", 0.5);
  auto_declare<std::string>("cmd_vel_topic", "/cmd_vel");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration TwistToCommandsController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration conf;
  conf.type = controller_interface::InterfaceConfiguration::Type::INDIVIDUAL;
  for (const auto & joint : wheel_joints_) {
    conf.names.push_back(joint + "/velocity");
  }
  for (const auto & joint : steering_joints_) {
    conf.names.push_back(joint + "/position");
  }
  return conf;
}

controller_interface::InterfaceConfiguration TwistToCommandsController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration conf;
  conf.type = controller_interface::InterfaceConfiguration::Type::NONE;
  return conf;
}

controller_interface::CallbackReturn TwistToCommandsController::on_configure(
  const rclcpp_lifecycle::State &)
{
  wheel_joints_ = get_node()->get_parameter("wheel_joints").as_string_array();
  steering_joints_ = get_node()->get_parameter("steering_joints").as_string_array();
  if (wheel_joints_.size() != 4 || steering_joints_.size() != 4) {
    RCLCPP_ERROR(get_node()->get_logger(),
      "Expected 4 wheel joints and 4 steering joints");
    return controller_interface::CallbackReturn::ERROR;
  }
  wheel_base_ = get_node()->get_parameter("wheel_base").as_double();
  track_width_ = get_node()->get_parameter("track_width").as_double();
  wheel_radius_ = get_node()->get_parameter("wheel_radius").as_double();
  max_steer_ = get_node()->get_parameter("max_steer").as_double();
  timeout_ = get_node()->get_parameter("twist_timeout").as_double();
  const auto cmd_topic = get_node()->get_parameter("cmd_vel_topic").as_string();

  sub_twist_ = get_node()->create_subscription<geometry_msgs::msg::Twist>(
    cmd_topic, rclcpp::SystemDefaultsQoS(),
    std::bind(&TwistToCommandsController::twistCb, this, std::placeholders::_1));

  last_twist_.linear.x = 0.0;
  last_twist_.angular.z = 0.0;
  last_twist_time_ = get_node()->now();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn TwistToCommandsController::on_activate(
  const rclcpp_lifecycle::State &)
{
  publishZeros();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn TwistToCommandsController::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  publishZeros();
  return controller_interface::CallbackReturn::SUCCESS;
}

void TwistToCommandsController::twistCb(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  last_twist_ = *msg;
  last_twist_time_ = get_node()->now();
}

void TwistToCommandsController::publishZeros()
{
  for (size_t i = 0; i < 4; ++i) {
    command_interfaces_[i].set_value(0.0);
    command_interfaces_[i + 4].set_value(0.0);
  }
}

controller_interface::return_type TwistToCommandsController::update(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  if ((get_node()->now() - last_twist_time_).seconds() > timeout_) {
    publishZeros();
    return controller_interface::return_type::OK;
  }

  const double vx = last_twist_.linear.x;
  const double vy = last_twist_.linear.y;
  const double wz = last_twist_.angular.z;

  const double hx = wheel_base_ * 0.5;
  const double hy = track_width_ * 0.5;

  struct Wheel { double x; double y; };
  const std::array<Wheel,4> wheels = {{
    { +hx, -hy },  // FL
    { +hx, +hy },  // FR
    { -hx, -hy },  // RL
    { -hx, +hy }   // RR
  }};

  std::array<double,4> steer{};
  std::array<double,4> speed{};

  for (size_t i = 0; i < wheels.size(); ++i) {
    const auto & w = wheels[i];
    const double vx_i = vx - wz * w.y;
    const double vy_i = vy + wz * w.x;
    double ang = std::atan2(vy_i, vx_i);
    double v_lin = std::hypot(vx_i, vy_i);
    double w_ang = (wheel_radius_ > 1e-9) ? (v_lin / wheel_radius_) : 0.0;
    if (std::abs(ang) > M_PI_2) {
      ang = ang > 0 ? ang - M_PI : ang + M_PI;
      w_ang = -w_ang;
    }
    steer[i] = std::clamp(ang, -max_steer_, max_steer_);
    speed[i] = w_ang;
  }

  for (size_t i = 0; i < 4; ++i) {
    command_interfaces_[i].set_value(speed[i]);
    command_interfaces_[i + 4].set_value(steer[i]);
  }

  return controller_interface::return_type::OK;
}

}  // namespace mr2_rover_control

PLUGINLIB_EXPORT_CLASS(mr2_rover_control::TwistToCommandsController, controller_interface::ControllerInterface)

