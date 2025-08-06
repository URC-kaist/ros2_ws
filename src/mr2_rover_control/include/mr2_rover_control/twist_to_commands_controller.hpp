#pragma once

#include <vector>
#include <string>
#include <array>

#include "controller_interface/controller_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "geometry_msgs/msg/twist.hpp"

namespace mr2_rover_control
{

class TwistToCommandsController : public controller_interface::ControllerInterface
{
public:
  controller_interface::CallbackReturn on_init() override;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  void twistCb(const geometry_msgs::msg::Twist::SharedPtr msg);
  void publishZeros();

  std::vector<std::string> wheel_joints_;
  std::vector<std::string> steering_joints_;

  double wheel_base_;
  double track_width_;
  double wheel_radius_;
  double max_steer_;
  double timeout_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_twist_;
  rclcpp::Time last_twist_time_;
  geometry_msgs::msg::Twist last_twist_;
};

}  // namespace mr2_rover_control

