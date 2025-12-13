#pragma once

#include "mr2_devices_sensors/homing_sensors.hpp"

#include <rclcpp/clock.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <std_msgs/msg/bool.hpp>

#include <cstdint>
#include <memory>
#include <string>

namespace mr2_devices_sensors {

class LimitSwitchDevice : public LimitSwitchDriver {
public:
  void configure(const hardware_interface::ComponentInfo &info,
                 rclcpp::Node *node) override;

  void process(const rclcpp::Time &now) override;

  void export_state(double *&, double *&, double *&) override;
  void export_command(double *&) override;
  void get_state(const double *&state, const double *&watchdog) override;

  const double *state_ptr() const { return &state_; }
  const double *watchdog_ptr() const { return &watchdog_state_; }

private:
  void on_frame(const can_frame &frame);

  rclcpp::Node *node_{nullptr};
  std::shared_ptr<CanBusManager> bus_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr state_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr fault_pub_;
  rclcpp::Logger logger_{rclcpp::get_logger("limit_switch_device")};
  rclcpp::Clock::SharedPtr ros_clock_;
  std::string state_topic_;
  std::string fault_topic_;
  std::string iface_;
  int bitrate_{1'000'000};
  uint32_t can_id_{0};
  bool active_high_{true};

  std::string state_name_;
  std::string fault_state_name_;
  std::string watchdog_state_name_;

  double state_{0.0};
  double fault_state_{0.0};
  bool frame_received_{false};
  bool timeout_logged_{false};
  double watchdog_state_{-1.0};
  double timeout_sec_{1.0};
  rclcpp::Time last_frame_time_;
  bool fault_reported_{false};

  template <typename MsgT>
  void safe_publish(const typename rclcpp::Publisher<MsgT>::SharedPtr &pub,
                    const MsgT &msg);
  bool can_publish() const;
};

} // namespace mr2_devices_sensors
