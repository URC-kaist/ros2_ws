#include "mr2_can_hardware_interface/homing_sensors/absolute_encoder_device.hpp"

#include "pluginlib/class_list_macros.hpp"

#include "rclcpp/clock.hpp"
#include "rclcpp/exceptions.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/utilities.hpp"
#include "std_msgs/msg/float64.hpp"

#include <cstdint>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <unordered_map>

namespace mr2_can_hardware_interface::sensors {

namespace {
constexpr uint32_t kStdIdMask = 0x7FFU;

constexpr uint8_t kEncFlagIndexSeen = 0x1;
constexpr uint8_t kEncFlagErrorLatched = 0x2;
constexpr uint8_t kEncFlagSensorFault = 0x4;

inline int32_t unpack_s24_be(const can_frame &frame) {
  int32_t raw = (static_cast<int32_t>(frame.data[0]) << 16) |
                (static_cast<int32_t>(frame.data[1]) << 8) |
                static_cast<int32_t>(frame.data[2]);
  if (raw & 0x800000) {
    raw |= ~0xFFFFFF;
  }
  return raw;
}

inline std::string require_param(
    const std::unordered_map<std::string, std::string> &params,
    const std::string &key) {
  auto it = params.find(key);
  if (it == params.end()) {
    throw std::runtime_error("Missing required parameter '" + key + "'");
  }
  return it->second;
}

inline uint32_t parse_can_id(const std::string &value) {
  std::size_t idx = 0;
  uint32_t id = 0;
  try {
    id = static_cast<uint32_t>(std::stoul(value, &idx, 0));
  } catch (const std::exception &) {
    throw std::runtime_error("Invalid CAN ID: " + value);
  }
  if (idx != value.size() || id > 0x1FFFFFFFU) {
    throw std::runtime_error("Invalid CAN ID: " + value);
  }
  return id;
}

template <typename T>
inline T parse_number(const std::unordered_map<std::string, std::string> &params,
                      const std::string &key, T def) {
  auto it = params.find(key);
  if (it == params.end()) {
    return def;
  }
  try {
    if constexpr (std::is_integral_v<T>) {
      return static_cast<T>(std::stoll(it->second));
    } else {
      return static_cast<T>(std::stod(it->second));
    }
  } catch (const std::exception &) {
    throw std::runtime_error("Invalid numeric value for " + key + ": " +
                             it->second);
  }
}

inline std::string resolve_topic(
    const std::unordered_map<std::string, std::string> &params,
    const std::string &key, const std::string &fallback) {
  auto it = params.find(key);
  if (it != params.end()) {
    return it->second;
  }
  return fallback;
}

inline std::string resolve_string(
    const std::unordered_map<std::string, std::string> &params,
    const std::string &key, const std::string &fallback) {
  auto it = params.find(key);
  if (it != params.end()) {
    return it->second;
  }
  return fallback;
}

} // namespace

void AbsoluteEncoderDevice::configure(const hardware_interface::ComponentInfo &info,
                                       rclcpp::Node *node) {
  node_ = node;

  iface_ = require_param(info.parameters, "can_iface");
  bitrate_ = parse_number<int>(info.parameters, "bitrate", 1'000'000);
  can_id_ = parse_can_id(require_param(info.parameters, "can_id"));
  ticks_per_rev_ = parse_number<double>(info.parameters, "ticks_per_rev", 4096.0);
  direction_ = parse_number<double>(info.parameters, "direction", 1.0);
  if (ticks_per_rev_ <= 0.0) {
    throw std::runtime_error("ticks_per_rev must be positive");
  }

  zero_offset_rad_ = -M_PI; // Fixed -180 degree offset

  state_name_ = require_param(info.parameters, "state_name");
  auto raw_it = info.parameters.find("raw_name");
  if (raw_it != info.parameters.end()) {
    raw_name_ = raw_it->second;
  }
  auto flags_it = info.parameters.find("flags_name");
  if (flags_it != info.parameters.end()) {
    flags_name_ = flags_it->second;
  }

  index_state_name_ = resolve_string(info.parameters, "index_state_name",
                                     state_name_ + "_index_seen");
  error_state_name_ = resolve_string(info.parameters, "error_state_name",
                                     state_name_ + "_error_latched");
  sensor_fault_state_name_ =
      resolve_string(info.parameters, "sensor_fault_state_name",
                     state_name_ + "_sensor_fault");

  logger_ = node->get_logger();
  ros_clock_ = node->get_clock();

  timeout_sec_ = parse_number<double>(info.parameters, "timeout_sec", 0.5);
  last_frame_time_ = ros_clock_->now();

  angle_topic_ = resolve_topic(info.parameters, "angle_topic",
                               "can_sensors/" + state_name_);
  angle_pub_ = node->create_publisher<std_msgs::msg::Float64>(
      angle_topic_, rclcpp::SensorDataQoS());

  if (!raw_name_.empty()) {
    raw_topic_ = resolve_topic(info.parameters, "raw_topic",
                               "can_sensors/" + raw_name_);
    raw_pub_ = node->create_publisher<std_msgs::msg::Float64>(
        raw_topic_, rclcpp::SensorDataQoS());
  }
  if (!flags_name_.empty()) {
    flags_topic_ = resolve_topic(info.parameters, "flags_topic",
                                 "can_sensors/" + flags_name_);
    flags_pub_ = node->create_publisher<std_msgs::msg::Float64>(
        flags_topic_, rclcpp::SensorDataQoS());
  }

  watchdog_state_name_ = state_name_ + "_watchdog";
  watchdog_state_ = -1.0;
  frame_received_ = false;

  bus_ = CanBusRegistry::get(iface_, bitrate_);
  if (!bus_) {
    throw std::runtime_error("Cannot open CAN bus: " + iface_);
  }

  add_filter(bus_, can_id_, kStdIdMask,
             [this](const can_frame &frame) { on_frame(frame); });
}

void AbsoluteEncoderDevice::process(const rclcpp::Time &now) {
  if (sensor_fault_state_ > 0.5) {
    watchdog_state_ = 1.0;
    return;
  }

  const double since_last = (now - last_frame_time_).seconds();
  const bool timed_out = since_last > timeout_sec_;

  if (timed_out) {
    watchdog_state_ = 1.0;
    if (!timeout_logged_) {
      RCLCPP_ERROR(logger_,
                   "Absolute encoder 0x%03X timed out (%.3f s > %.3f s)",
                   can_id_, since_last, timeout_sec_);
      timeout_logged_ = true;
    }
  } else {
    watchdog_state_ = frame_received_ ? 0.0 : -1.0;
    if (timeout_logged_) {
      RCLCPP_WARN(logger_,
                  "Absolute encoder 0x%03X feedback recovered after timeout.",
                  can_id_);
      timeout_logged_ = false;
    }
  }
}

void AbsoluteEncoderDevice::export_state(double *&, double *&, double *&) {}

void AbsoluteEncoderDevice::export_command(double *&) {}

void AbsoluteEncoderDevice::get_state(const double *&angle, const double *&watchdog) {
  angle = &angle_rad_;
  watchdog = &watchdog_state_;
}

void AbsoluteEncoderDevice::on_frame(const can_frame &frame) {
  const uint8_t flags_byte = frame.data[3];
  flags_ = static_cast<double>(flags_byte);

  const bool sensor_fault = (flags_byte & kEncFlagSensorFault) != 0;
  const bool index_seen = (flags_byte & kEncFlagIndexSeen) != 0;
  const bool error_latched = (flags_byte & kEncFlagErrorLatched) != 0;

  index_seen_state_ = index_seen ? 1.0 : 0.0;
  error_latched_state_ = error_latched ? 1.0 : 0.0;
  sensor_fault_state_ = sensor_fault ? 1.0 : 0.0;

  if (index_seen && !index_seen_reported_) {
    RCLCPP_INFO(logger_, "Encoder 0x%03X reported valid samples.", can_id_);
    index_seen_reported_ = true;
  }

  if (error_latched && !error_latched_reported_) {
    RCLCPP_ERROR(logger_,
                 "Encoder 0x%03X latched a sensor error flag.", can_id_);
    error_latched_reported_ = true;
  }

  if (sensor_fault) {
    if (!sensor_fault_reported_) {
      RCLCPP_WARN(logger_,
                  "Encoder 0x%03X reported a sensor fault; ignoring samples "
                  "until it clears.",
                  can_id_);
      sensor_fault_reported_ = true;
    }
  } else if (sensor_fault_reported_) {
    RCLCPP_INFO(logger_, "Encoder 0x%03X sensor fault cleared.", can_id_);
    sensor_fault_reported_ = false;
  }

  if (!sensor_fault) {
    const int32_t raw_q24 = unpack_s24_be(frame);
    const double counts = static_cast<double>(raw_q24) /
                          static_cast<double>(1 << 12);
    raw_counts_ = counts;

    const double revolutions = counts / ticks_per_rev_;
    angle_rad_ = direction_ * revolutions * 2.0 * M_PI - zero_offset_rad_;

    if (angle_pub_ && can_publish()) {
      std_msgs::msg::Float64 msg;
      msg.data = angle_rad_;
      safe_publish(angle_pub_, msg);
    }
    if (raw_pub_ && can_publish()) {
      std_msgs::msg::Float64 msg;
      msg.data = raw_counts_;
      safe_publish(raw_pub_, msg);
    }
  }

  if (flags_pub_ && can_publish()) {
    std_msgs::msg::Float64 msg;
    msg.data = flags_;
    safe_publish(flags_pub_, msg);
  }

  if (ros_clock_) {
    last_frame_time_ = ros_clock_->now();
  }
  frame_received_ = frame_received_ || index_seen;
  watchdog_state_ = sensor_fault ? 1.0 : 0.0;
  timeout_logged_ = false;
}

template<typename MsgT>
void AbsoluteEncoderDevice::safe_publish(
    const typename rclcpp::Publisher<MsgT>::SharedPtr &pub,
    const MsgT &msg) {
  if (!pub || !can_publish()) {
    return;
  }
  try {
    pub->publish(msg);
  } catch (const rclcpp::exceptions::RCLError &ex) {
    RCLCPP_WARN_ONCE(logger_,
                     "Absolute encoder publisher inactive during shutdown: %s",
                     ex.what());
  }
}

bool AbsoluteEncoderDevice::can_publish() const {
  if (!node_) {
    return false;
  }
  auto base = node_->get_node_base_interface();
  if (!base) {
    return false;
  }
  auto context = base->get_context();
  return context && context->is_valid() && rclcpp::ok(context);
}

} // namespace mr2_can_hardware_interface::sensors

PLUGINLIB_EXPORT_CLASS(mr2_can_hardware_interface::sensors::AbsoluteEncoderDevice,
                       CanDevice)
