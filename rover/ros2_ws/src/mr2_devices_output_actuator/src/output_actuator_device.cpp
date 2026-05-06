#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "builtin_interfaces/msg/time.hpp"
#include "mr2_can_bus_core/can_bus_registry.hpp"
#include "mr2_can_bus_core/can_device.hpp"
#include "mr2_devices_output_actuator/msg/actuator_config_status.hpp"
#include "mr2_devices_output_actuator/msg/output_angle_status.hpp"
#include "mr2_devices_output_actuator/msg/output_velocity_status.hpp"
#include "mr2_devices_output_actuator/msg/runtime_diagnostic.hpp"
#include "mr2_devices_output_actuator/msg/travel_limits_status.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/qos.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>

namespace mr2_devices_output_actuator {

using ActuatorConfigStatusMsg =
    mr2_devices_output_actuator::msg::ActuatorConfigStatus;
using OutputAngleStatusMsg =
    mr2_devices_output_actuator::msg::OutputAngleStatus;
using OutputVelocityStatusMsg =
    mr2_devices_output_actuator::msg::OutputVelocityStatus;
using RuntimeDiagnosticMsg =
    mr2_devices_output_actuator::msg::RuntimeDiagnostic;
using TravelLimitsStatusMsg =
    mr2_devices_output_actuator::msg::TravelLimitsStatus;

namespace {

constexpr uint16_t kAngleCommandBase = 0x200;
constexpr uint16_t kAngleStatusBase = 0x400;
constexpr uint16_t kVelocityCommandBase = 0x210;
constexpr uint16_t kVelocityStatusBase = 0x410;
constexpr uint16_t kLimitsStatusBase = 0x420;
constexpr uint16_t kConfigStatusBase = 0x430;
constexpr uint16_t kProfileCommandBase = 0x220;
constexpr uint16_t kPowerCommandBase = 0x230;
constexpr uint16_t kRuntimeDiagBase = 0x5F0;
constexpr uint32_t kStandardIdMask = 0x7FF;
constexpr uint16_t kMinReleasedNodeId = 1;
constexpr uint16_t kMaxReleasedNodeId = 15;
constexpr uint8_t kRuntimeDiagMagic = 0xFB;
constexpr double kPi = 3.14159265358979323846;
constexpr double kRadToDeg = 180.0 / kPi;
constexpr double kDegToRad = kPi / 180.0;
constexpr double kAngleToleranceRad = 0.001 * kDegToRad;

enum class CommandChannel { Angle, Velocity };

enum class Profile : uint8_t {
  VelocityOnly = 0,
  As5600 = 1,
  TmagLut = 2,
  DirectInput = 3,
};

enum class ControlMode : uint8_t {
  OutputAngle = 1,
  OutputVelocity = 2,
};

bool parse_bool(const std::string &value) {
  return value == "true" || value == "True" || value == "1";
}

Profile parse_profile(const std::string &value) {
  if (value == "VelocityOnly") {
    return Profile::VelocityOnly;
  }
  if (value == "As5600") {
    return Profile::As5600;
  }
  if (value == "TmagLut") {
    return Profile::TmagLut;
  }
  if (value == "DirectInput") {
    return Profile::DirectInput;
  }
  throw std::runtime_error("Unsupported desired_profile '" + value + "'");
}

CommandChannel parse_command_channel(const std::string &value) {
  if (value == "angle") {
    return CommandChannel::Angle;
  }
  if (value == "velocity") {
    return CommandChannel::Velocity;
  }
  throw std::runtime_error("Unsupported command_channel '" + value + "'");
}

int32_t decode_i32_le(const can_frame &frame, size_t offset) {
  return static_cast<int32_t>(
      static_cast<uint32_t>(frame.data[offset]) |
      (static_cast<uint32_t>(frame.data[offset + 1]) << 8) |
      (static_cast<uint32_t>(frame.data[offset + 2]) << 16) |
      (static_cast<uint32_t>(frame.data[offset + 3]) << 24));
}

void encode_i32_le(can_frame &frame, int32_t value) {
  frame.data[0] = static_cast<uint8_t>(value & 0xFF);
  frame.data[1] = static_cast<uint8_t>((value >> 8) & 0xFF);
  frame.data[2] = static_cast<uint8_t>((value >> 16) & 0xFF);
  frame.data[3] = static_cast<uint8_t>((value >> 24) & 0xFF);
}

int32_t clamp_i32(long long value) {
  if (value > std::numeric_limits<int32_t>::max()) {
    return std::numeric_limits<int32_t>::max();
  }
  if (value < std::numeric_limits<int32_t>::min()) {
    return std::numeric_limits<int32_t>::min();
  }
  return static_cast<int32_t>(value);
}

int32_t rad_to_mdeg(double rad) {
  return clamp_i32(std::llround(rad * kRadToDeg * 1000.0));
}

int32_t rad_s_to_mdeg_s(double rad_s) {
  return clamp_i32(std::llround(rad_s * kRadToDeg * 1000.0));
}

double mdeg_to_rad(int32_t mdeg) {
  return (static_cast<double>(mdeg) / 1000.0) * kDegToRad;
}

double mdeg_s_to_rad_s(int32_t mdeg_s) {
  return (static_cast<double>(mdeg_s) / 1000.0) * kDegToRad;
}

bool profile_requires_output_feedback(Profile profile) {
  return profile != Profile::VelocityOnly;
}

} // namespace

class OutputActuatorDevice : public CanDevice {
public:
  void configure(const hardware_interface::ComponentInfo &info,
                 rclcpp::Node *node) override {
    node_ = node;
    logger_ = node_ ? node_->get_logger()
                    : rclcpp::get_logger("output_actuator_device");

    const auto iface_it = info.parameters.find("can_iface");
    if (iface_it == info.parameters.end()) {
      throw std::runtime_error("Missing can_iface parameter");
    }
    iface_ = iface_it->second;

    const auto node_id_it = info.parameters.find("node_id");
    if (node_id_it == info.parameters.end()) {
      throw std::runtime_error("Missing node_id parameter");
    }
    node_id_ = static_cast<uint16_t>(std::stoi(node_id_it->second));
    if (node_id_ < kMinReleasedNodeId || node_id_ > kMaxReleasedNodeId) {
      throw std::runtime_error(
          "node_id must be in 1..15 for the NoFW 0x10-spaced frame family");
    }

    const auto channel_it = info.parameters.find("command_channel");
    if (channel_it != info.parameters.end()) {
      channel_ = parse_command_channel(channel_it->second);
    } else if (!info.command_interfaces.empty() &&
               info.command_interfaces[0].name ==
                   hardware_interface::HW_IF_VELOCITY) {
      channel_ = CommandChannel::Velocity;
    }
    require_limits_status_ = channel_ == CommandChannel::Angle;

    const auto profile_it = info.parameters.find("desired_profile");
    if (profile_it != info.parameters.end()) {
      desired_profile_ = parse_profile(profile_it->second);
    } else {
      desired_profile_ = channel_ == CommandChannel::Velocity
                             ? Profile::VelocityOnly
                             : Profile::As5600;
    }

    const auto auto_arm_it = info.parameters.find("auto_arm");
    if (auto_arm_it != info.parameters.end()) {
      auto_arm_ = parse_bool(auto_arm_it->second);
    }
    const auto require_limits_it = info.parameters.find("require_limits_status");
    if (require_limits_it != info.parameters.end()) {
      require_limits_status_ = parse_bool(require_limits_it->second);
    }
    const auto require_config_it = info.parameters.find("require_config_status");
    if (require_config_it != info.parameters.end()) {
      require_config_status_ = parse_bool(require_config_it->second);
    }
    const auto timeout_it = info.parameters.find("activation_timeout_ms");
    if (timeout_it != info.parameters.end()) {
      activation_timeout_ms_ = std::max(1, std::stoi(timeout_it->second));
    }
    const auto arm_attempts_it = info.parameters.find("activation_arm_attempts");
    if (arm_attempts_it != info.parameters.end()) {
      activation_arm_attempts_ = std::max(1, std::stoi(arm_attempts_it->second));
    }

    const auto actuator_it = info.parameters.find("actuator");
    const std::string default_status_topic_base =
        "output_actuator/" +
        (actuator_it != info.parameters.end() ? actuator_it->second
                                              : info.name);
    const auto status_topic_base_it =
        info.parameters.find("status_topic_base");
    status_topic_base_ = status_topic_base_it != info.parameters.end()
                             ? status_topic_base_it->second
                             : default_status_topic_base;

    const auto expected_min_it =
        info.parameters.find("expected_output_min_deg");
    if (expected_min_it != info.parameters.end()) {
      expected_output_min_rad_ =
          std::stod(expected_min_it->second) * kDegToRad;
    }
    const auto expected_max_it =
        info.parameters.find("expected_output_max_deg");
    if (expected_max_it != info.parameters.end()) {
      expected_output_max_rad_ =
          std::stod(expected_max_it->second) * kDegToRad;
    }

    bus_ = CanBusRegistry::get(iface_);
    if (!bus_) {
      throw std::runtime_error("Cannot open CAN bus");
    }

    add_filter(bus_, kAngleStatusBase + node_id_, kStandardIdMask,
               [this](const can_frame &f) { on_angle_status(f); });
    add_filter(bus_, kVelocityStatusBase + node_id_, kStandardIdMask,
               [this](const can_frame &f) { on_velocity_status(f); });
    add_filter(bus_, kLimitsStatusBase + node_id_, kStandardIdMask,
               [this](const can_frame &f) { on_limits_status(f); });
    add_filter(bus_, kConfigStatusBase + node_id_, kStandardIdMask,
               [this](const can_frame &f) { on_config_status(f); });
    add_filter(bus_, kRuntimeDiagBase + node_id_, kStandardIdMask,
               [this](const can_frame &f) { on_runtime_diag(f); });

    if (node_) {
      const auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
      angle_status_pub_ =
          node_->create_publisher<OutputAngleStatusMsg>(
              status_topic_base_ + "/angle_status", qos);
      velocity_status_pub_ =
          node_->create_publisher<OutputVelocityStatusMsg>(
              status_topic_base_ + "/velocity_status", qos);
      travel_limits_status_pub_ =
          node_->create_publisher<TravelLimitsStatusMsg>(
              status_topic_base_ + "/travel_limits_status", qos);
      config_status_pub_ =
          node_->create_publisher<ActuatorConfigStatusMsg>(
              status_topic_base_ + "/config_status", qos);
      runtime_diagnostic_pub_ =
          node_->create_publisher<RuntimeDiagnosticMsg>(
              status_topic_base_ + "/runtime_diagnostic", qos);
    }

    last_integrate_time_ns_ = clock_.now().nanoseconds();
  }

  bool on_activate() override {
    active_ = false;

    if (!auto_arm_) {
      active_ = true;
      return true;
    }

    if (!wait_for([this] { return diag_seen_.load(); }, "runtime diagnostic")) {
      return false;
    }

    if (armed_) {
      send_power_command(false);
      if (!wait_for([this] { return diag_seen_.load() && !armed_.load(); },
                    "disarmed diagnostic")) {
        return false;
      }
    }

    if (stored_profile_.load() != static_cast<uint8_t>(desired_profile_) ||
        active_profile_.load() != static_cast<uint8_t>(desired_profile_)) {
      config_status_seen_ = false;
      send_profile_command(desired_profile_);
      if (!wait_for([this] {
            return diag_seen_.load() &&
                   stored_profile_.load() ==
                       static_cast<uint8_t>(desired_profile_) &&
                   active_profile_.load() ==
                       static_cast<uint8_t>(desired_profile_);
          },
                    "desired profile diagnostic")) {
        RCLCPP_ERROR(logger_, "Actuator %u profile select result: %u", node_id_,
                     profile_select_result_.load());
        return false;
      }
    }

    if (require_limits_status_ &&
        !wait_for([this] { return limits_status_seen_.load(); },
                  "limits status")) {
      return false;
    }
    if (require_config_status_ &&
        !wait_for([this] { return config_status_seen_.load(); },
                  "config status")) {
      return false;
    }
    if (!validate_config_status()) {
      return false;
    }
    if (!validate_runtime_diag_ready("before arm")) {
      return false;
    }

    if (channel_ == CommandChannel::Angle) {
      if (!wait_for([this] { return angle_status_seen_.load(); },
                    "angle status before arm")) {
        return false;
      }
      hold_position_rad_ = position_rad_;
      transmit_angle(hold_position_rad_);
    } else {
      transmit_velocity(0.0);
    }

    if (!arm_with_confirmation()) {
      return false;
    }
    if (!validate_runtime_diag_ready("after arm")) {
      return false;
    }

    active_ = true;
    return true;
  }

  void on_deactivate() override {
    active_ = false;
    if (channel_ == CommandChannel::Velocity) {
      transmit_velocity(0.0);
    }
    if (auto_arm_) {
      send_power_command(false);
    }
  }

  void process(const rclcpp::Time &) override {
    integrate_position(clock_.now().nanoseconds());

    if (!active_.load()) {
      return;
    }
    if (auto_arm_ && !armed_.load()) {
      return;
    }

    if (channel_ == CommandChannel::Angle) {
      double desired = desired_command_;
      if (!std::isfinite(desired)) {
        desired = std::isfinite(hold_position_rad_) ? hold_position_rad_
                                                    : position_rad_;
      } else {
        hold_position_rad_ = desired;
      }
      if (std::isfinite(desired)) {
        transmit_angle(desired);
      }
    } else {
      const double desired = std::isfinite(desired_command_) ? desired_command_
                                                             : 0.0;
      transmit_velocity(desired);
    }
  }

  void export_state(double *&position, double *&velocity,
                    double *&effort) override {
    if (!std::isfinite(position_rad_)) {
      position_rad_ = 0.0;
    }
    if (!std::isfinite(velocity_rad_s_)) {
      velocity_rad_s_ = 0.0;
    }
    position = &position_rad_;
    velocity = &velocity_rad_s_;
    effort_dummy_ = 0.0;
    effort = &effort_dummy_;
  }

  void export_command(double *&command) override { command = &desired_command_; }

private:
  bool valid_frame(const can_frame &frame, uint8_t expected_dlc) const {
    return (frame.can_id & CAN_EFF_FLAG) == 0 &&
           (frame.can_id & CAN_RTR_FLAG) == 0 && frame.can_dlc == expected_dlc;
  }

  builtin_interfaces::msg::Time stamp_now() const {
    const auto now = node_ ? node_->now() : rclcpp::Clock(RCL_SYSTEM_TIME).now();
    const int64_t ns = now.nanoseconds();
    builtin_interfaces::msg::Time stamp;
    stamp.sec = static_cast<int32_t>(ns / 1000000000LL);
    stamp.nanosec = static_cast<uint32_t>(ns % 1000000000LL);
    return stamp;
  }

  uint8_t expected_control_mode() const {
    return static_cast<uint8_t>(channel_ == CommandChannel::Angle
                                    ? ControlMode::OutputAngle
                                    : ControlMode::OutputVelocity);
  }

  bool validate_config_status() const {
    if (!config_status_seen_.load()) {
      return true;
    }

    if (stored_output_encoder_type_.load() !=
        static_cast<uint8_t>(desired_profile_)) {
      RCLCPP_ERROR(logger_,
                   "Actuator %u config profile mismatch: stored=%u expected=%u",
                   node_id_, stored_output_encoder_type_.load(),
                   static_cast<uint8_t>(desired_profile_));
      return false;
    }
    if (config_default_control_mode_.load() != expected_control_mode()) {
      RCLCPP_ERROR(logger_,
                   "Actuator %u config default mode mismatch: got=%u expected=%u",
                   node_id_, config_default_control_mode_.load(),
                   expected_control_mode());
      return false;
    }
    if (channel_ == CommandChannel::Angle && !config_angle_mode_enabled_.load()) {
      RCLCPP_ERROR(logger_, "Actuator %u config reports angle mode disabled",
                   node_id_);
      return false;
    }
    if (channel_ == CommandChannel::Velocity &&
        !config_velocity_mode_enabled_.load()) {
      RCLCPP_ERROR(logger_, "Actuator %u config reports velocity mode disabled",
                   node_id_);
      return false;
    }
    if (expected_output_min_rad_.has_value()) {
      if (!limits_status_seen_.load()) {
        RCLCPP_ERROR(logger_,
                     "Actuator %u missing travel limits status for expected min",
                     node_id_);
        return false;
      }
      if (std::abs(output_min_rad_ - *expected_output_min_rad_) >
          kAngleToleranceRad) {
        RCLCPP_ERROR(logger_,
                     "Actuator %u min travel mismatch: got=%.3f deg expected=%.3f deg",
                     node_id_, output_min_rad_ * kRadToDeg,
                     *expected_output_min_rad_ * kRadToDeg);
        return false;
      }
    }
    if (expected_output_max_rad_.has_value()) {
      if (!limits_status_seen_.load()) {
        RCLCPP_ERROR(logger_,
                     "Actuator %u missing travel limits status for expected max",
                     node_id_);
        return false;
      }
      if (std::abs(output_max_rad_ - *expected_output_max_rad_) >
          kAngleToleranceRad) {
        RCLCPP_ERROR(logger_,
                     "Actuator %u max travel mismatch: got=%.3f deg expected=%.3f deg",
                     node_id_, output_max_rad_ * kRadToDeg,
                     *expected_output_max_rad_ * kRadToDeg);
        return false;
      }
    }
    return true;
  }

  bool validate_runtime_diag_ready(const char *phase) const {
    if (!diag_seen_.load()) {
      RCLCPP_ERROR(logger_, "Actuator %u has no runtime diagnostic", node_id_);
      return false;
    }
    if (runtime_diag_magic_.load() != kRuntimeDiagMagic) {
      RCLCPP_ERROR(logger_,
                   "Actuator %u invalid runtime diagnostic magic during %s: 0x%02X",
                   node_id_, phase, runtime_diag_magic_.load());
      return false;
    }
    if (stored_profile_.load() != static_cast<uint8_t>(desired_profile_) ||
        active_profile_.load() != static_cast<uint8_t>(desired_profile_)) {
      RCLCPP_ERROR(
          logger_,
          "Actuator %u profile mismatch during %s: stored=%u active=%u expected=%u",
          node_id_, phase, stored_profile_.load(), active_profile_.load(),
          static_cast<uint8_t>(desired_profile_));
      return false;
    }
    if (diag_default_control_mode_.load() != expected_control_mode()) {
      RCLCPP_ERROR(
          logger_,
          "Actuator %u diagnostic default mode mismatch during %s: got=%u expected=%u",
          node_id_, phase, diag_default_control_mode_.load(),
          expected_control_mode());
      return false;
    }
    if (channel_ == CommandChannel::Angle && !diag_angle_mode_enabled_.load()) {
      RCLCPP_ERROR(logger_,
                   "Actuator %u diagnostic reports angle mode disabled during %s",
                   node_id_, phase);
      return false;
    }
    if (channel_ == CommandChannel::Velocity &&
        !diag_velocity_mode_enabled_.load()) {
      RCLCPP_ERROR(
          logger_,
          "Actuator %u diagnostic reports velocity mode disabled during %s",
          node_id_, phase);
      return false;
    }
    if (!trusted_foc_calibration_valid_.load()) {
      RCLCPP_ERROR(logger_,
                   "Actuator %u trusted FOC calibration is invalid during %s",
                   node_id_, phase);
      return false;
    }
    if (profile_requires_output_feedback(desired_profile_) &&
        !trusted_output_calibration_valid_.load()) {
      RCLCPP_ERROR(
          logger_,
          "Actuator %u trusted output calibration is invalid during %s",
          node_id_, phase);
      return false;
    }
    if (runtime_fault_.load() != 0) {
      RCLCPP_ERROR(logger_,
                   "Actuator %u reports runtime fault %u during %s", node_id_,
                   runtime_fault_.load(), phase);
      return false;
    }
    if (need_calibration_.load()) {
      RCLCPP_ERROR(logger_, "Actuator %u reports calibration required during %s",
                   node_id_, phase);
      return false;
    }
    const bool expected_feedback_required =
        profile_requires_output_feedback(desired_profile_);
    if (output_feedback_required_.load() != expected_feedback_required) {
      RCLCPP_ERROR(
          logger_,
          "Actuator %u output feedback requirement mismatch during %s: got=%u expected=%u",
          node_id_, phase, output_feedback_required_.load() ? 1 : 0,
          expected_feedback_required ? 1 : 0);
      return false;
    }
    return true;
  }

  bool wait_for(const std::function<bool()> &predicate,
                const char *description) {
    const auto start = std::chrono::steady_clock::now();
    const auto timeout = std::chrono::milliseconds(activation_timeout_ms_);
    while (std::chrono::steady_clock::now() - start < timeout) {
      if (predicate()) {
        return true;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    if (predicate()) {
      return true;
    }
    RCLCPP_ERROR(logger_, "Actuator %u timed out waiting for %s",
                 node_id_, description);
    return false;
  }

  bool arm_with_confirmation() {
    for (int attempt = 1; attempt <= activation_arm_attempts_; ++attempt) {
      send_power_command(true);
      if (wait_for([this] { return diag_seen_.load() && armed_.load(); },
                   "armed diagnostic")) {
        if (attempt > 1) {
          RCLCPP_INFO(logger_, "Actuator %u armed after %d attempts", node_id_,
                      attempt);
        }
        return true;
      }
      if (attempt < activation_arm_attempts_) {
        RCLCPP_WARN(logger_,
                    "Actuator %u arm attempt %d/%d failed; retrying", node_id_,
                    attempt, activation_arm_attempts_);
      }
    }

    RCLCPP_ERROR(logger_, "Actuator %u failed to arm after %d attempts",
                 node_id_, activation_arm_attempts_);
    return false;
  }

  void send_profile_command(Profile profile) {
    can_frame frame{};
    frame.can_id = kProfileCommandBase + node_id_;
    frame.can_dlc = 1;
    frame.data[0] = static_cast<uint8_t>(profile);
    send(frame, bus_);
  }

  void send_power_command(bool arm) {
    can_frame frame{};
    frame.can_id = kPowerCommandBase + node_id_;
    frame.can_dlc = 1;
    frame.data[0] = arm ? 1 : 0;
    send(frame, bus_);
  }

  void transmit_angle(double position_rad) {
    can_frame frame{};
    frame.can_id = kAngleCommandBase + node_id_;
    frame.can_dlc = 4;
    encode_i32_le(frame, rad_to_mdeg(position_rad));
    send(frame, bus_);
  }

  void transmit_velocity(double velocity_rad_s) {
    can_frame frame{};
    frame.can_id = kVelocityCommandBase + node_id_;
    frame.can_dlc = 4;
    encode_i32_le(frame, rad_s_to_mdeg_s(velocity_rad_s));
    send(frame, bus_);
  }

  void integrate_position(int64_t now_ns) {
    if (channel_ != CommandChannel::Velocity) {
      last_integrate_time_ns_ = now_ns;
      return;
    }
    if (last_integrate_time_ns_ > 0 && std::isfinite(velocity_rad_s_)) {
      const double dt = (now_ns - last_integrate_time_ns_) * 1e-9;
      if (dt > 0.0 && dt < 1.0) {
        position_rad_ += velocity_rad_s_ * dt;
      }
    }
    last_integrate_time_ns_ = now_ns;
  }

  void on_angle_status(const can_frame &frame) {
    if (!valid_frame(frame, 4)) {
      return;
    }
    const int32_t mdeg = decode_i32_le(frame, 0);
    position_rad_ = mdeg_to_rad(mdeg);
    angle_status_seen_ = true;
    if (!std::isfinite(hold_position_rad_)) {
      hold_position_rad_ = position_rad_;
    }
    if (angle_status_pub_) {
      OutputAngleStatusMsg msg;
      msg.stamp = stamp_now();
      msg.node_id = node_id_;
      msg.angle_mdeg = mdeg;
      msg.angle_rad = position_rad_;
      std::copy(frame.data, frame.data + 4, msg.raw_data.begin());
      angle_status_pub_->publish(msg);
    }
  }

  void on_velocity_status(const can_frame &frame) {
    if (!valid_frame(frame, 4)) {
      return;
    }
    const int32_t mdeg_s = decode_i32_le(frame, 0);
    velocity_rad_s_ = mdeg_s_to_rad_s(mdeg_s);
    velocity_status_seen_ = true;
    if (velocity_status_pub_) {
      OutputVelocityStatusMsg msg;
      msg.stamp = stamp_now();
      msg.node_id = node_id_;
      msg.velocity_mdeg_s = mdeg_s;
      msg.velocity_rad_s = velocity_rad_s_;
      std::copy(frame.data, frame.data + 4, msg.raw_data.begin());
      velocity_status_pub_->publish(msg);
    }
  }

  void on_limits_status(const can_frame &frame) {
    if (!valid_frame(frame, 8)) {
      return;
    }
    const int32_t output_min_mdeg = decode_i32_le(frame, 0);
    const int32_t output_max_mdeg = decode_i32_le(frame, 4);
    output_min_rad_ = mdeg_to_rad(output_min_mdeg);
    output_max_rad_ = mdeg_to_rad(output_max_mdeg);
    limits_status_seen_ = true;
    if (travel_limits_status_pub_) {
      TravelLimitsStatusMsg msg;
      msg.stamp = stamp_now();
      msg.node_id = node_id_;
      msg.output_min_mdeg = output_min_mdeg;
      msg.output_max_mdeg = output_max_mdeg;
      msg.output_min_rad = output_min_rad_;
      msg.output_max_rad = output_max_rad_;
      std::copy(frame.data, frame.data + 8, msg.raw_data.begin());
      travel_limits_status_pub_->publish(msg);
    }
  }

  void on_config_status(const can_frame &frame) {
    if (!valid_frame(frame, 8)) {
      return;
    }
    const int32_t gear_ratio_milli = decode_i32_le(frame, 0);
    gear_ratio_ = static_cast<double>(gear_ratio_milli) / 1000.0;
    stored_output_encoder_type_ = frame.data[4];
    config_default_control_mode_ = frame.data[5];
    config_velocity_mode_enabled_ = (frame.data[6] & 0x01) != 0;
    config_angle_mode_enabled_ = (frame.data[6] & 0x02) != 0;
    config_status_seen_ = true;
    if (config_status_pub_) {
      ActuatorConfigStatusMsg msg;
      msg.stamp = stamp_now();
      msg.node_id = node_id_;
      msg.gear_ratio_milli = gear_ratio_milli;
      msg.gear_ratio = gear_ratio_;
      msg.stored_output_encoder_type = stored_output_encoder_type_.load();
      msg.default_control_mode = config_default_control_mode_.load();
      msg.velocity_mode_enabled = config_velocity_mode_enabled_.load();
      msg.output_angle_mode_enabled = config_angle_mode_enabled_.load();
      msg.reserved = frame.data[7];
      std::copy(frame.data, frame.data + 8, msg.raw_data.begin());
      config_status_pub_->publish(msg);
    }
  }

  void on_runtime_diag(const can_frame &frame) {
    if (!valid_frame(frame, 8)) {
      return;
    }
    runtime_diag_magic_ = frame.data[0];
    stored_profile_ = frame.data[1];
    active_profile_ = frame.data[2];
    diag_default_control_mode_ = frame.data[3];
    diag_velocity_mode_enabled_ = (frame.data[4] & 0x01) != 0;
    diag_angle_mode_enabled_ = (frame.data[4] & 0x02) != 0;
    trusted_foc_calibration_valid_ = (frame.data[4] & 0x04) != 0;
    trusted_output_calibration_valid_ = (frame.data[4] & 0x08) != 0;
    runtime_fault_ = frame.data[5];
    need_calibration_ = (frame.data[6] & 0x01) != 0;
    profile_select_result_ = (frame.data[6] >> 4) & 0x0F;
    output_feedback_required_ = (frame.data[7] & 0x01) != 0;
    armed_ = (frame.data[7] & 0x02) != 0;
    diag_seen_ = true;
    if (runtime_diagnostic_pub_) {
      RuntimeDiagnosticMsg msg;
      msg.stamp = stamp_now();
      msg.node_id = node_id_;
      msg.magic = runtime_diag_magic_.load();
      msg.stored_output_encoder_type = stored_profile_.load();
      msg.active_output_encoder_type = active_profile_.load();
      msg.default_control_mode = diag_default_control_mode_.load();
      msg.velocity_mode_enabled = diag_velocity_mode_enabled_.load();
      msg.output_angle_mode_enabled = diag_angle_mode_enabled_.load();
      msg.trusted_foc_calibration_valid =
          trusted_foc_calibration_valid_.load();
      msg.trusted_output_calibration_valid =
          trusted_output_calibration_valid_.load();
      msg.calibration_load_status = (frame.data[4] >> 4) & 0x03;
      msg.runtime_fault = runtime_fault_.load();
      msg.need_calibration = need_calibration_.load();
      msg.profile_select_result = profile_select_result_.load();
      msg.output_feedback_required = output_feedback_required_.load();
      msg.power_stage_armed = armed_.load();
      std::copy(frame.data, frame.data + 8, msg.raw_data.begin());
      runtime_diagnostic_pub_->publish(msg);
    }
  }

  std::shared_ptr<CanBusManager> bus_;
  rclcpp::Node *node_{nullptr};
  rclcpp::Logger logger_{rclcpp::get_logger("output_actuator_device")};
  rclcpp::Clock clock_{RCL_STEADY_TIME};
  std::string iface_;
  std::string status_topic_base_;
  uint16_t node_id_{0};

  CommandChannel channel_{CommandChannel::Angle};
  Profile desired_profile_{Profile::As5600};
  std::atomic<uint8_t> stored_profile_{
      static_cast<uint8_t>(Profile::VelocityOnly)};
  std::atomic<uint8_t> active_profile_{
      static_cast<uint8_t>(Profile::VelocityOnly)};
  bool auto_arm_{true};
  bool require_limits_status_{true};
  bool require_config_status_{true};
  int activation_timeout_ms_{1500};
  int activation_arm_attempts_{3};

  std::atomic_bool active_{false};
  std::atomic_bool diag_seen_{false};
  std::atomic_bool limits_status_seen_{false};
  std::atomic_bool config_status_seen_{false};
  std::atomic_bool angle_status_seen_{false};
  std::atomic_bool velocity_status_seen_{false};
  std::atomic_bool need_calibration_{true};
  std::atomic_bool armed_{false};
  std::atomic_bool config_velocity_mode_enabled_{false};
  std::atomic_bool config_angle_mode_enabled_{false};
  std::atomic_bool diag_velocity_mode_enabled_{false};
  std::atomic_bool diag_angle_mode_enabled_{false};
  std::atomic_bool trusted_foc_calibration_valid_{false};
  std::atomic_bool trusted_output_calibration_valid_{false};
  std::atomic_bool output_feedback_required_{false};
  std::atomic<uint8_t> profile_select_result_{0};
  std::atomic<uint8_t> stored_output_encoder_type_{0};
  std::atomic<uint8_t> config_default_control_mode_{0};
  std::atomic<uint8_t> diag_default_control_mode_{0};
  std::atomic<uint8_t> runtime_diag_magic_{0};
  std::atomic<uint8_t> runtime_fault_{0};

  int64_t last_integrate_time_ns_{0};
  double position_rad_{0.0};
  double velocity_rad_s_{0.0};
  double desired_command_{std::numeric_limits<double>::quiet_NaN()};
  double hold_position_rad_{std::numeric_limits<double>::quiet_NaN()};
  double effort_dummy_{0.0};
  double output_min_rad_{std::numeric_limits<double>::quiet_NaN()};
  double output_max_rad_{std::numeric_limits<double>::quiet_NaN()};
  double gear_ratio_{std::numeric_limits<double>::quiet_NaN()};
  std::optional<double> expected_output_min_rad_;
  std::optional<double> expected_output_max_rad_;

  rclcpp::Publisher<OutputAngleStatusMsg>::SharedPtr angle_status_pub_;
  rclcpp::Publisher<OutputVelocityStatusMsg>::SharedPtr velocity_status_pub_;
  rclcpp::Publisher<TravelLimitsStatusMsg>::SharedPtr
      travel_limits_status_pub_;
  rclcpp::Publisher<ActuatorConfigStatusMsg>::SharedPtr config_status_pub_;
  rclcpp::Publisher<RuntimeDiagnosticMsg>::SharedPtr runtime_diagnostic_pub_;
};

} // namespace mr2_devices_output_actuator

PLUGINLIB_EXPORT_CLASS(mr2_devices_output_actuator::OutputActuatorDevice,
                       CanDevice)
