#include "mr2_can_bus_core/can_device.hpp"

#include "pluginlib/class_list_macros.hpp"

#include "rclcpp/clock.hpp"
#include "rclcpp/exceptions.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/utilities.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int8.hpp"

#include <cctype>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>
#include <unordered_map>

namespace mr2_devices_sensors {

namespace {
constexpr uint32_t kStdIdMask = 0x7FFU;

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

inline bool parse_bool(const std::unordered_map<std::string, std::string> &params,
                       const std::string &key, bool def) {
  auto it = params.find(key);
  if (it == params.end()) {
    return def;
  }
  const std::string &value = it->second;
  if (value == "true" || value == "1" || value == "True") {
    return true;
  }
  if (value == "false" || value == "0" || value == "False") {
    return false;
  }
  throw std::runtime_error("Invalid boolean for " + key + ": " + value);
}

inline int parse_int(const std::unordered_map<std::string, std::string> &params,
                     const std::string &key, int def) {
  auto it = params.find(key);
  if (it == params.end()) {
    return def;
  }
  try {
    return std::stoi(it->second);
  } catch (const std::exception &) {
    throw std::runtime_error("Invalid integer for " + key + ": " + it->second);
  }
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
    throw std::runtime_error("Invalid numeric for " + key + ": " + it->second);
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

class LimitSwitchDevice : public CanDevice {
public:
  void configure(const hardware_interface::ComponentInfo &info,
                 rclcpp::Node *node) override {
    node_ = node;

    iface_ = require_param(info.parameters, "can_iface");
    bitrate_ = parse_int(info.parameters, "bitrate", 1'000'000);
    can_id_ = parse_can_id(require_param(info.parameters, "can_id"));
    active_high_ = parse_bool(info.parameters, "active_high", true);

    state_name_ = require_param(info.parameters, "state_name");
    auto edge_it = info.parameters.find("edge_name");
    if (edge_it != info.parameters.end()) {
      edge_name_ = edge_it->second;
    }

    logger_ = node->get_logger();

    const std::string state_topic = make_sensor_topic(state_name_);
    state_topic_ = state_topic;
    state_pub_ = node->create_publisher<std_msgs::msg::Bool>(
        state_topic, rclcpp::SensorDataQoS());

    if (!edge_name_.empty()) {
      const std::string edge_topic = make_sensor_topic(edge_name_);
      edge_topic_ = edge_topic;
      edge_pub_ = node->create_publisher<std_msgs::msg::Int8>(
          edge_topic, rclcpp::SensorDataQoS());
    }

    bus_ = CanBusRegistry::get(iface_, bitrate_);
    if (!bus_) {
      throw std::runtime_error("Cannot open CAN bus: " + iface_);
    }

    add_filter(bus_, can_id_, kStdIdMask,
               [this](const can_frame &frame) { on_frame(frame); });

    timeout_sec_ = parse_double(info.parameters, "timeout_sec", 0.5);
    ros_clock_ = node->get_clock();
    last_frame_time_ = ros_clock_->now();
    watchdog_state_name_ = state_name_ + "_watchdog";
    watchdog_state_ = -1.0;
    frame_received_ = false;
  }

  void process(const rclcpp::Time &now) override {
    const double since_last = (now - last_frame_time_).seconds();
    const bool timed_out = since_last > timeout_sec_;

    if (timed_out) {
      watchdog_state_ = 1.0;
      if (!timeout_warned_) {
        RCLCPP_ERROR(logger_,
                     "Limit switch 0x%03X timed out (%.3f s > %.3f s)",
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
                    "Limit switch 0x%03X recovered after timeout.",
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
    states.emplace_back(state_name_, &state_);
    if (!edge_name_.empty()) {
      states.emplace_back(edge_name_, &edge_);
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
    const bool pressed = (frame.data[0] & 0x1) != 0;
    const double logical = pressed ? 1.0 : 0.0;
    const double mapped_state =
        active_high_ ? logical : (logical > 0.5 ? 0.0 : 1.0);

    if (!reserved_warned_) {
      for (int i = 1; i < 8; ++i) {
        if (frame.data[i] != 0) {
          reserved_warned_ = true;
          RCLCPP_WARN(logger_,
                      "Limit switch 0x%03X reserved byte %d non-zero (%u)",
                      can_id_, i, static_cast<unsigned int>(frame.data[i]));
          break;
        }
      }
    }

    double edge_value = 0.0;
    if (last_state_valid_) {
      if (mapped_state > 0.5 && last_state_ <= 0.5) {
        edge_value = 1.0;
      } else if (mapped_state <= 0.5 && last_state_ > 0.5) {
        edge_value = -1.0;
      }
    } else {
      last_state_valid_ = true;
    }

    state_ = mapped_state;
    last_state_ = mapped_state;
    edge_ = edge_value;

    if (ros_clock_) {
      last_frame_time_ = ros_clock_->now();
    }
    frame_received_ = true;
    watchdog_state_ = 0.0;
    timeout_warned_ = false;

    if (state_pub_ && can_publish()) {
      std_msgs::msg::Bool msg;
      msg.data = state_ > 0.5;
      safe_publish(state_pub_, msg);
    }
    if (edge_pub_ && (edge_value != 0.0) && can_publish()) {
      std_msgs::msg::Int8 msg;
      msg.data = static_cast<int8_t>(edge_value > 0.0 ? 1 : -1);
      safe_publish(edge_pub_, msg);
    }
  }

  rclcpp::Node *node_{nullptr};
  std::shared_ptr<CanBusManager> bus_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr state_pub_;
  rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr edge_pub_;
  rclcpp::Logger logger_{rclcpp::get_logger("limit_switch_device")};
  rclcpp::Clock::SharedPtr ros_clock_;
  std::string state_topic_;
  std::string edge_topic_;
  std::string iface_;
  int bitrate_{1'000'000};
  uint32_t can_id_{0};
  bool active_high_{true};

  std::string state_name_;
  std::string edge_name_;
  std::string watchdog_state_name_;

  double state_{0.0};
  double edge_{0.0};
  double last_state_{0.0};
  bool last_state_valid_{false};
  bool reserved_warned_{false};
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
                       "Limit switch publisher inactive during shutdown: %s",
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

PLUGINLIB_EXPORT_CLASS(mr2_devices_sensors::LimitSwitchDevice, CanDevice)
