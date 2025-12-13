#pragma once

#include "mr2_can_hardware_interface/homing_sensors/homing_sensors.hpp"

#include <rclcpp/clock.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <std_msgs/msg/float64.hpp>

#include <cstdint>
#include <memory>
#include <string>

namespace mr2_can_hardware_interface::sensors {

class AbsoluteEncoderDevice : public AbsoluteEncoderDriver {
public:
  void configure(const hardware_interface::ComponentInfo &info,
                 rclcpp::Node *node) override;

  void process(const rclcpp::Time &now) override;

  void export_state(double *&, double *&, double *&) override;
  void export_command(double *&) override;
  void get_state(const double *&angle, const double *&watchdog) override;
  const double *angle_ptr() const override { return &angle_rad_; }
  const double *watchdog_ptr() const override { return &watchdog_state_; }

private:
  void on_frame(const can_frame &frame);

  rclcpp::Node *node_{nullptr};
  std::shared_ptr<CanBusManager> bus_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr angle_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr raw_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr flags_pub_;
  rclcpp::Logger logger_{rclcpp::get_logger("absolute_encoder_device")};
  rclcpp::Clock::SharedPtr ros_clock_;
  std::string angle_topic_;
  std::string raw_topic_;
  std::string flags_topic_;
  std::string iface_;
  int bitrate_{1'000'000};
  uint32_t can_id_{0};
  double ticks_per_rev_{4096.0};
  double direction_{1.0};
  double zero_offset_rad_{0.0};

  std::string state_name_;
  std::string raw_name_;
  std::string flags_name_;
  std::string watchdog_state_name_;
  std::string index_state_name_;
  std::string error_state_name_;
  std::string sensor_fault_state_name_;

  double angle_rad_{0.0};
  double raw_counts_{0.0};
  double flags_{0.0};
  double index_seen_state_{0.0};
  double error_latched_state_{0.0};
  double sensor_fault_state_{0.0};
  bool frame_received_{false};
  bool timeout_logged_{false};
  double watchdog_state_{-1.0};
  double timeout_sec_{0.5};
  rclcpp::Time last_frame_time_;
  bool error_latched_reported_{false};
  bool sensor_fault_reported_{false};
  bool index_seen_reported_{false};

  template <typename MsgT>
  void safe_publish(const typename rclcpp::Publisher<MsgT>::SharedPtr &pub,
                    const MsgT &msg);
  bool can_publish() const;
};

} // namespace mr2_can_hardware_interface::sensors
