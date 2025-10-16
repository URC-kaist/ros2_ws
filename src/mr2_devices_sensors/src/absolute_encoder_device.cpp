#include "mr2_can_bus_core/can_device.hpp"

#include "pluginlib/class_list_macros.hpp"

#include "rclcpp/clock.hpp"
#include "rclcpp/exceptions.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/utilities.hpp"
#include "std_msgs/msg/float64.hpp"

#include <cmath>
#include <cstdint>
#include <cctype>
#include <limits>
#include <stdexcept>
#include <string>
#include <unordered_map>

namespace mr2_devices_sensors {

namespace {
constexpr uint32_t kStdIdMask = 0x7FFU;

inline int32_t unpack_s24_be(const can_frame &frame) {
  int32_t raw = (static_cast<int32_t>(frame.data[0]) << 16) |
                (static_cast<int32_t>(frame.data[1]) << 8) |
                static_cast<int32_t>(frame.data[2]);
  if (raw & 0x800000) {
    raw |= ~0xFFFFFF;
  }
  return raw;
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

inline double parse_double(const std::unordered_map<std::string, std::string> &params,
                           const std::string &key, double def) {
  auto it = params.find(key);
  if (it == params.end()) {
    return def;
  }
  try {
    return std::stod(it->second);
  } catch (const std::exception &) {
    throw std::runtime_error("Invalid numeric value for " + key + ": " + it->second);
  }
}

inline std::string make_sensor_topic(const std::string &name) {
  if (name.empty()) {
    return {};
  }
  std::string sanitized;
  sanitized.reserve(name.size());
  for (char ch : name) {
    unsigned char uc = static_cast<unsigned char>(ch);
    if (std::isalnum(uc) || ch == '/' || ch == '_') {
      sanitized.push_back(ch);
    } else {
      sanitized.push_back('_');
    }
  }
  if (!sanitized.empty() && sanitized.front() == '/') {
    return sanitized;
  }
  return std::string("can_sensors/") + sanitized;
}

} // namespace

class AbsoluteEncoderDevice : public CanDevice {
public:
  void configure(const hardware_interface::ComponentInfo &info,
                 rclcpp::Node *node) override {
    node_ = node;

    iface_ = require_param(info.parameters, "can_iface");
    bitrate_ = static_cast<int>(parse_double(info.parameters, "bitrate", 1'000'000));
    can_id_ = parse_can_id(require_param(info.parameters, "can_id"));
    ticks_per_rev_ = parse_double(info.parameters, "ticks_per_rev", 4096.0);
    direction_ = parse_double(info.parameters, "direction", 1.0);

    const double zero_deg = parse_double(info.parameters, "zero_offset_deg", 0.0);
    const double zero_rad = parse_double(info.parameters, "zero_offset_rad", zero_deg * M_PI / 180.0);
    zero_offset_rad_ = zero_rad;

    state_name_ = require_param(info.parameters, "state_name");
    auto raw_it = info.parameters.find("raw_name");
    if (raw_it != info.parameters.end()) {
      raw_name_ = raw_it->second;
    }
    auto flags_it = info.parameters.find("flags_name");
    if (flags_it != info.parameters.end()) {
      flags_name_ = flags_it->second;
    }

    logger_ = node->get_logger();
    ros_clock_ = node->get_clock();

    timeout_sec_ = parse_double(info.parameters, "timeout_sec", 0.5);
    last_frame_time_ = ros_clock_->now();

    const std::string angle_topic = make_sensor_topic(state_name_);
    angle_topic_ = angle_topic;
    angle_pub_ = node->create_publisher<std_msgs::msg::Float64>(
        angle_topic, rclcpp::SensorDataQoS());

    if (!raw_name_.empty()) {
      const std::string raw_topic = make_sensor_topic(raw_name_);
      raw_topic_ = raw_topic;
      raw_pub_ = node->create_publisher<std_msgs::msg::Float64>(
          raw_topic, rclcpp::SensorDataQoS());
    }
    if (!flags_name_.empty()) {
      const std::string flags_topic = make_sensor_topic(flags_name_);
      flags_topic_ = flags_topic;
      flags_pub_ = node->create_publisher<std_msgs::msg::Float64>(
          flags_topic, rclcpp::SensorDataQoS());
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

  void process(const rclcpp::Time &now) override {
    const double since_last = (now - last_frame_time_).seconds();
    const bool timed_out = since_last > timeout_sec_;

    if (timed_out) {
      watchdog_state_ = 1.0;
      if (!timeout_warned_) {
        RCLCPP_ERROR(logger_,
                     "Absolute encoder 0x%03X timed out (%.3f s > %.3f s)",
                     can_id_, since_last, timeout_sec_);
        timeout_warned_ = true;
      }
      timeout_active_ = true;
    } else {
      if (frame_received_) {
        watchdog_state_ = 0.0;
      } else {
        watchdog_state_ = -1.0;
      }
      if (timeout_active_) {
        RCLCPP_WARN(logger_,
                    "Absolute encoder 0x%03X feedback recovered after timeout.",
                    can_id_);
        timeout_active_ = false;
      }
      timeout_warned_ = false;
    }
  }

  void export_state(std::vector<double *> &, std::vector<double *> &,
                    std::vector<double *> &) override {}

  void export_command(std::vector<double *> &) override {}

  void export_named_states(
      std::vector<std::pair<std::string, double *>> &states) override {
    states.emplace_back(state_name_, &angle_rad_);
    if (!raw_name_.empty()) {
      states.emplace_back(raw_name_, &raw_counts_);
    }
    if (!flags_name_.empty()) {
      states.emplace_back(flags_name_, &flags_);
    }
    states.emplace_back(watchdog_state_name_, &watchdog_state_);
  }

private:
  static std::string require_param(const std::unordered_map<std::string, std::string> &params,
                                   const std::string &key) {
    auto it = params.find(key);
    if (it == params.end()) {
      throw std::runtime_error("Missing required parameter '" + key + "'");
    }
    return it->second;
  }

  void on_frame(const can_frame &frame) {
    const int32_t raw_q24 = unpack_s24_be(frame);
    const double counts = static_cast<double>(raw_q24) / static_cast<double>(1 << 12);
    raw_counts_ = counts;

    const double revolutions = counts / ticks_per_rev_;
    angle_rad_ = direction_ * revolutions * 2.0 * M_PI - zero_offset_rad_;

    if (!flags_name_.empty()) {
      flags_ = static_cast<double>(frame.data[3]);
    }

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
    if (flags_pub_ && can_publish()) {
      std_msgs::msg::Float64 msg;
      msg.data = flags_;
      safe_publish(flags_pub_, msg);
    }

    if (ros_clock_) {
      last_frame_time_ = ros_clock_->now();
    }
    frame_received_ = true;
    watchdog_state_ = 0.0;
    timeout_warned_ = false;
  }

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

  double angle_rad_{0.0};
  double raw_counts_{0.0};
  double flags_{0.0};
  bool frame_received_{false};
  bool timeout_warned_{false};
  bool timeout_active_{false};
  double watchdog_state_{-1.0};
  double timeout_sec_{0.5};
  rclcpp::Time last_frame_time_;

  template<typename MsgT>
  void safe_publish(const typename rclcpp::Publisher<MsgT>::SharedPtr &pub,
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

  bool can_publish() const {
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
};

} // namespace mr2_devices_sensors

PLUGINLIB_EXPORT_CLASS(mr2_devices_sensors::AbsoluteEncoderDevice, CanDevice)
