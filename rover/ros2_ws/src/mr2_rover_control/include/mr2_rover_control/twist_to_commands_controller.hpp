#pragma once

#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "mr2_action_interface/msg/mission_status.hpp"
#include "mr2_rover_control/four_wheel_steering_solver.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace mr2_rover_control {

class TwistToCommandsController
    : public controller_interface::ControllerInterface {
public:
  controller_interface::CallbackReturn on_init() override;

  controller_interface::InterfaceConfiguration
  command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration
  state_interface_configuration() const override;

  controller_interface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State &previous_state) override;
  controller_interface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &previous_state) override;
  controller_interface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

  controller_interface::return_type
  update(const rclcpp::Time &time, const rclcpp::Duration &period) override;

private:
  static constexpr uint8_t kMissionStateRunning = 1;

  void twistNominalCb(const geometry_msgs::msg::Twist::SharedPtr msg);
  void twistRunningCb(const geometry_msgs::msg::Twist::SharedPtr msg);
  void missionStatusCb(
      const mr2_action_interface::msg::MissionStatus::SharedPtr msg);
  void publishZeros();
  bool fillWheelStates(std::array<double, 4> &wheel_ang_vel,
                       std::array<double, 4> &steer_angle) const;
  void publishOdom(const std::array<double, 4> &wheel_ang_vel,
                   const std::array<double, 4> &steer_angle,
                   const rclcpp::Time &stamp);
  std::array<std::array<double, 2>, 4> wheelPositions() const;
  double applyRateLimit(double target, double prev, double rate_limit,
                        double dt) const;

  std::vector<std::string> wheel_joints_;
  std::vector<std::string> steering_joints_;

  double wheel_base_;
  double track_width_;
  double wheel_radius_;
  double max_steer_;
  double timeout_;
  double odom_publish_rate_;
  double solver_error_alpha_;
  double solver_gain_k_;
  double rate_limit_vx_;
  double rate_limit_vy_;
  double rate_limit_wz_;
  double steering_error_ratio_rad_;
  std::string odom_frame_id_;
  std::string base_frame_id_;

  FourWheelSteeringSolver::Config solver_cfg_;
  std::optional<FourWheelSteeringSolver> solver_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_twist_nominal_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_twist_running_;
  rclcpp::Subscription<mr2_action_interface::msg::MissionStatus>::SharedPtr
      sub_mission_status_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Time last_twist_time_;
  rclcpp::Time last_odom_pub_time_;
  geometry_msgs::msg::Twist last_twist_;
  geometry_msgs::msg::Twist limited_twist_;

  std::optional<uint8_t> mission_state_;
};

} // namespace mr2_rover_control
