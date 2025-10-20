/*
 * Mock AK servo node.
 *
 * This ROS 2 node generates AK servo style CAN frames so that the single
 * joint demo can run without physical hardware. It integrates a simple
 * position controller, publishes limit-switch feedback, and can optionally
 * emulate an absolute encoder.
 *
 * Parameters:
 *   can_iface (string, default "vcan0")
 *     CAN interface used for TX/RX.
 *   motor_id (int, default 4)
 *     Servo identifier encoded into outgoing frames.
 *   update_rate_hz (double, default 200.0)
 *     Control loop rate for integrating position.
 *   max_velocity (double, default 10.0)
 *     Maximum speed (rad/s) applied during integration.
 *   effort_gain (double, default 4.0)
 *     Proportional gain for the effort/current estimate.
 *   max_effort_amp (double, default 6.0)
 *     Saturation limit for the effort/current estimate.
 *   position_limit (double, default +inf)
 *     Symmetric position limit in radians; set <= 0.0 to disable clamping.
 *   max_integration_dt (double, default 0.05)
 *     Maximum integration step size to cope with timer jitter.
 *   initial_position (double, default 0.0)
 *     Initial joint position in radians (clamped to the configured limits).
 *   limit_switch_enabled (bool, default false)
 *     Enable publication of limit switch state frames.
 *   limit_switch_can_id (int, default 0x181)
 *     CAN identifier used for limit switch frames.
 *   limit_switch_active_high (bool, default true)
 *     Whether a logical 1 indicates that the switch is pressed.
 *   limit_switch_trigger_position (double, default NaN)
 *     Position threshold in radians that activates the switch (if enabled).
 *   limit_switch_trigger_when_below (bool, default true)
 *     Switch activates when position <= threshold (otherwise >= threshold).
 *   absolute_encoder_enabled (bool, default false)
 *     Enable publication of absolute encoder frames.
 *   absolute_encoder_can_id (int, default 384 / 0x180)
 *     CAN identifier used for encoder frames (decimal input expected).
 *   absolute_encoder_ticks_per_rev (double, default 4096.0)
 *     Resolution of the encoder, used when quantising to counts.
 *   absolute_encoder_direction (double, default 1.0)
 *     Multiplier (+1 or -1) that lets the encoder be flipped in software.
 *   absolute_encoder_zero_offset (double, default 0.0)
 *     Mechanical offset in radians that aligns the encoder with the desired
 * home.
 */

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <mutex>
#include <rclcpp/logging.hpp>
#include <stdexcept>
#include <string>

#include <linux/can.h>

#include "mr2_can_bus_core/can_bus_manager.hpp"
#include "mr2_can_bus_core/can_bus_registry.hpp"
#include "rclcpp/rclcpp.hpp"

namespace {
constexpr uint32_t kMaxCanId = 0x1FFFFFFF;
constexpr double kDirectionEpsilon = 1e-6;
constexpr double kTwoPi = 6.28318530717958647692; // 2 * pi
constexpr double kQ24Scale = static_cast<double>(1 << 12);
constexpr double kMaxQ24 = static_cast<double>(0x7FFFFF);
constexpr double kMinQ24 = static_cast<double>(-0x800000);
} // namespace

class MockAkServoNode : public rclcpp::Node {
public:
  MockAkServoNode() : rclcpp::Node("mock_ak_servo") {
    load_parameters();
    initialise_bus();
  }

private:
  using rclcpp::Node::declare_parameter;

  void load_parameters() {
    can_iface_ = declare_parameter<std::string>("can_iface", "vcan0");
    servo_id_ = declare_parameter<int>("motor_id", 4);
    update_rate_hz_ = declare_parameter<double>("update_rate_hz", 200.0);
    if (update_rate_hz_ <= 0.0) {
      throw std::runtime_error("update_rate_hz must be positive");
    }

    max_velocity_rad_s_ = declare_parameter<double>("max_velocity", 10.0);
    if (max_velocity_rad_s_ <= 0.0) {
      throw std::runtime_error("max_velocity must be positive");
    }

    effort_gain_ = declare_parameter<double>("effort_gain", 4.0);
    max_effort_amp_ = declare_parameter<double>("max_effort_amp", 6.0);
    position_limit_ = declare_parameter<double>(
        "position_limit", std::numeric_limits<double>::infinity());

    max_integration_dt_ = declare_parameter<double>("max_integration_dt", 0.05);
    if (max_integration_dt_ < 0.0) {
      throw std::runtime_error("max_integration_dt cannot be negative");
    }

    initial_position_ = declare_parameter<double>("initial_position", 0.0);

    limit_switch_enabled_ =
        declare_parameter<bool>("limit_switch_enabled", false);
    limit_switch_can_id_ = declare_parameter<int>("limit_switch_can_id", 0x181);
    if (limit_switch_can_id_ < 0 ||
        static_cast<uint32_t>(limit_switch_can_id_) > kMaxCanId) {
      throw std::runtime_error(
          "limit_switch_can_id out of range (0..0x1FFFFFFF)");
    }

    limit_switch_active_high_ =
        declare_parameter<bool>("limit_switch_active_high", true);
    limit_switch_trigger_position_ =
        declare_parameter<double>("limit_switch_trigger_position",
                                  std::numeric_limits<double>::quiet_NaN());
    limit_switch_trigger_when_below_ =
        declare_parameter<bool>("limit_switch_trigger_when_below", true);

    absolute_encoder_enabled_ =
        declare_parameter<bool>("absolute_encoder_enabled", false);
    absolute_encoder_can_id_ = static_cast<uint32_t>(
        declare_parameter<int>("absolute_encoder_can_id", 0x180));
    if (absolute_encoder_can_id_ > kMaxCanId) {
      throw std::runtime_error(
          "absolute_encoder_can_id out of range (0..0x1FFFFFFF)");
    }

    absolute_encoder_ticks_per_rev_ =
        declare_parameter<double>("absolute_encoder_ticks_per_rev", 4096.0);
    if (absolute_encoder_ticks_per_rev_ <= 0.0 ||
        !std::isfinite(absolute_encoder_ticks_per_rev_)) {
      throw std::runtime_error(
          "absolute_encoder_ticks_per_rev must be positive and finite");
    }

    absolute_encoder_direction_ =
        declare_parameter<double>("absolute_encoder_direction", 1.0);
    if (!std::isfinite(absolute_encoder_direction_) ||
        std::fabs(absolute_encoder_direction_) < kDirectionEpsilon) {
      throw std::runtime_error(
          "absolute_encoder_direction must be finite and non-zero");
    }

    absolute_encoder_zero_offset_ =
        declare_parameter<double>("absolute_encoder_zero_offset", 0.0);

    if (servo_id_ < 0 || static_cast<uint32_t>(servo_id_) > kMaxCanId) {
      throw std::runtime_error("motor_id out of range (0..0x1FFFFFFF)");
    }
  }

  void initialise_bus() {
    bus_ = CanBusRegistry::get(can_iface_);
    if (!bus_) {
      throw std::runtime_error("Failed to acquire CAN bus on interface '" +
                               can_iface_ + "'");
    }

    // Listen for control commands directed at this servo.
    const uint32_t command_filter_id =
        0x00000400u | static_cast<uint32_t>(servo_id_);
    bus_->register_listener(
        command_filter_id, kMaxCanId,
        [this](const struct can_frame &frame) { this->handle_command(frame); });

    // Ensure the commanded target starts at the current (limited) position.
    {
      std::lock_guard<std::mutex> lock(state_mtx_);
      position_rad_ = apply_position_limits(initial_position_);
      target_position_rad_ = position_rad_;
    }

    // Start the update loop.
    const auto period = std::chrono::duration<double>(1.0 / update_rate_hz_);
    timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        std::bind(&MockAkServoNode::update, this));

    last_time_ = std::chrono::steady_clock::now();
    RCLCPP_INFO(get_logger(), "Mock AK servo started on %s (motor_id=%d)",
                can_iface_.c_str(), servo_id_);
  }

  void handle_command(const struct can_frame &frame) {
    if ((frame.can_id & CAN_EFF_FLAG) == 0 || frame.can_dlc < 4) {
      return;
    }

    const int32_t raw = (static_cast<int32_t>(frame.data[0]) << 24) |
                        (static_cast<int32_t>(frame.data[1]) << 16) |
                        (static_cast<int32_t>(frame.data[2]) << 8) |
                        (static_cast<int32_t>(frame.data[3]));

    const double target_deg = static_cast<double>(raw) / 1e4;
    const double target_rad = target_deg * M_PI / 180.0;
    const double limited_target_rad = apply_position_limits(target_rad);

    {
      std::lock_guard<std::mutex> lock(state_mtx_);
      target_position_rad_ = limited_target_rad;
    }

    RCLCPP_INFO(get_logger(), " target=%.4f rad  limited=%.4f rad", target_rad,
                limited_target_rad);
  }

  void update() {
    const auto now = std::chrono::steady_clock::now();
    double dt = std::chrono::duration<double>(now - last_time_).count();
    last_time_ = now;
    if (dt <= 0.0) {
      dt = 1.0 / update_rate_hz_;
    } else if (max_integration_dt_ > 0.0) {
      dt = std::min(dt, max_integration_dt_);
    }

    double target;
    double position;
    {
      std::lock_guard<std::mutex> lock(state_mtx_);

      target = target_position_rad_;
      position = position_rad_;

      const double error = target - position;
      const double max_step = max_velocity_rad_s_ * dt;
      const double step = std::clamp(error, -max_step, max_step);
      position_rad_ = apply_position_limits(position_rad_ + step);

      if (position_limit_ > 0.0 &&
          std::fabs(position_rad_) >= position_limit_ - 1e-6 &&
          std::fabs(error) > max_step) {
        velocity_rad_s_ = 0.0;
      } else {
        velocity_rad_s_ = step / dt;
      }
      effort_amp_ =
          std::clamp(error * effort_gain_, -max_effort_amp_, max_effort_amp_);

      position = position_rad_;
      target_position_rad_ = apply_position_limits(target_position_rad_);
    }

    publish_status(position, velocity_rad_s_, effort_amp_);
    publish_limit_switch(position);
    publish_absolute_encoder(position);
  }

  void publish_status(double position, double velocity, double effort) {
    struct can_frame status {};
    status.can_id =
        (0x00002900u | static_cast<uint32_t>(servo_id_)) | CAN_EFF_FLAG;
    status.can_dlc = 8;

    const int16_t pos_10deg =
        static_cast<int16_t>(std::lround(position * 180.0 / M_PI * 10.0));
    const double rpm = velocity * 30.0 / M_PI;
    const int16_t vel_10rpm = static_cast<int16_t>(std::lround(rpm * 10.0));
    const int16_t cur_01amp = static_cast<int16_t>(std::lround(effort * 100.0));

    status.data[0] = static_cast<uint8_t>((pos_10deg >> 8) & 0xFF);
    status.data[1] = static_cast<uint8_t>(pos_10deg & 0xFF);
    status.data[2] = static_cast<uint8_t>((vel_10rpm >> 8) & 0xFF);
    status.data[3] = static_cast<uint8_t>(vel_10rpm & 0xFF);
    status.data[4] = static_cast<uint8_t>((cur_01amp >> 8) & 0xFF);
    status.data[5] = static_cast<uint8_t>(cur_01amp & 0xFF);
    status.data[6] = 0;
    status.data[7] = 0;

    bus_->enqueue_tx(status);
  }

  void publish_limit_switch(double position) {
    if (!limit_switch_enabled_ ||
        !std::isfinite(limit_switch_trigger_position_)) {
      return;
    }

    const bool triggered = limit_switch_trigger_when_below_
                               ? position <= limit_switch_trigger_position_
                               : position >= limit_switch_trigger_position_;
    const bool pressed = limit_switch_active_high_ ? triggered : !triggered;

    struct can_frame frame {};
    frame.can_id = static_cast<uint32_t>(limit_switch_can_id_) & kMaxCanId;
    frame.can_dlc = 1;
    frame.data[0] = pressed ? 0x1 : 0x0;
    bus_->enqueue_tx(frame);
  }

  void publish_absolute_encoder(double position) {
    if (!absolute_encoder_enabled_) {
      return;
    }

    const double counts = (position + absolute_encoder_zero_offset_) *
                          absolute_encoder_ticks_per_rev_ /
                          (kTwoPi * absolute_encoder_direction_);
    const double raw_q24 = std::clamp(counts * kQ24Scale, kMinQ24, kMaxQ24);
    const int32_t raw = static_cast<int32_t>(std::llround(raw_q24));

    struct can_frame frame {};
    frame.can_id = absolute_encoder_can_id_;
    frame.can_dlc = 4;
    frame.data[0] = static_cast<uint8_t>((raw >> 16) & 0xFF);
    frame.data[1] = static_cast<uint8_t>((raw >> 8) & 0xFF);
    frame.data[2] = static_cast<uint8_t>(raw & 0xFF);
    frame.data[3] = 0x00;
    bus_->enqueue_tx(frame);
  }

  double apply_position_limits(double value) const {
    if (position_limit_ <= 0.0 || !std::isfinite(position_limit_)) {
      return value;
    }
    return std::clamp(value, -position_limit_, position_limit_);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<CanBusManager> bus_;

  std::string can_iface_;
  int servo_id_{4};
  double update_rate_hz_{200.0};
  double max_velocity_rad_s_{10.0};
  double effort_gain_{4.0};
  double max_effort_amp_{6.0};
  double position_limit_{std::numeric_limits<double>::infinity()};
  double max_integration_dt_{0.05};
  double initial_position_{0.0};

  bool limit_switch_enabled_{false};
  int limit_switch_can_id_{0x181};
  bool limit_switch_active_high_{true};
  double limit_switch_trigger_position_{
      std::numeric_limits<double>::quiet_NaN()};
  bool limit_switch_trigger_when_below_{true};

  bool absolute_encoder_enabled_{false};
  uint32_t absolute_encoder_can_id_{0x180};
  double absolute_encoder_ticks_per_rev_{4096.0};
  double absolute_encoder_direction_{1.0};
  double absolute_encoder_zero_offset_{0.0};

  std::mutex state_mtx_;
  double target_position_rad_{0.0};
  double position_rad_{0.0};
  double velocity_rad_s_{0.0};
  double effort_amp_{0.0};

  std::chrono::steady_clock::time_point last_time_{};
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<MockAkServoNode>());
  } catch (const std::exception &ex) {
    RCLCPP_FATAL(rclcpp::get_logger("mock_ak_servo"), "Unhandled exception: %s",
                 ex.what());
    rclcpp::shutdown();
    return EXIT_FAILURE;
  }

  rclcpp::shutdown();
  return EXIT_SUCCESS;
}
