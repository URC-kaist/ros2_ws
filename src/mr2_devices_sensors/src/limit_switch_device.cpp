#include "mr2_can_bus_core/can_device.hpp"

#include "pluginlib/class_list_macros.hpp"

#include "rclcpp/clock.hpp"
#include "rclcpp/exceptions.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/utilities.hpp"
#include "std_msgs/msg/bool.hpp"

#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <unordered_map>

namespace mr2_devices_sensors {

namespace {
constexpr uint32_t kStdIdMask = 0x7FFU;
constexpr uint8_t kFaultBit = 0x1;

std::string
require_param(const std::unordered_map<std::string, std::string> &params,
              const std::string &key) {
  auto it = params.find(key);
  if (it == params.end()) {
    throw std::runtime_error("Missing required parameter '" + key + "'");
  }
  return it->second;
}

uint32_t parse_can_id(const std::string &value) {
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

bool parse_bool(const std::unordered_map<std::string, std::string> &params,
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

template <typename T>
T parse_number(const std::unordered_map<std::string, std::string> &params,
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
    throw std::runtime_error("Invalid numeric for " + key + ": " + it->second);
  }
}

std::string
resolve_topic(const std::unordered_map<std::string, std::string> &params,
              const std::string &key, const std::string &default_topic) {
  auto it = params.find(key);
  if (it != params.end()) {
    return it->second;
  }
  return default_topic;
}

} // namespace

class LimitSwitchDevice : public CanDevice {
public:
  void configure(const hardware_interface::ComponentInfo &info,
                 rclcpp::Node *node) override {
    node_ = node;

    iface_ = require_param(info.parameters, "can_iface");
    bitrate_ = parse_number<int>(info.parameters, "bitrate", 1'000'000);
    can_id_ = parse_can_id(require_param(info.parameters, "can_id"));
    active_high_ = parse_bool(info.parameters, "active_high", true);

    state_name_ = require_param(info.parameters, "state_name");
    fault_state_name_ = resolve_topic(info.parameters, "fault_state_name",
                                      state_name_ + "_fault");

    logger_ = node->get_logger();

    state_topic_ = resolve_topic(info.parameters, "state_topic",
                                 "can_sensors/" + state_name_);
    state_pub_ = node->create_publisher<std_msgs::msg::Bool>(
        state_topic_, rclcpp::SensorDataQoS());

    fault_topic_ = resolve_topic(info.parameters, "fault_topic",
                                 "can_sensors/" + fault_state_name_);
    if (!fault_topic_.empty()) {
      fault_pub_ = node->create_publisher<std_msgs::msg::Bool>(
          fault_topic_, rclcpp::SensorDataQoS());
    }

    bus_ = CanBusRegistry::get(iface_, bitrate_);
    if (!bus_) {
      throw std::runtime_error("Cannot open CAN bus: " + iface_);
    }

    add_filter(bus_, can_id_, kStdIdMask,
               [this](const can_frame &frame) { on_frame(frame); });

    timeout_sec_ = parse_number<double>(info.parameters, "timeout_sec", 1.0);
    ros_clock_ = node->get_clock();
    last_frame_time_ = ros_clock_->now();
    watchdog_state_name_ = state_name_ + "_watchdog";
    watchdog_state_ = -1.0;
    frame_received_ = false;
  }

  void process(const rclcpp::Time &now) override {
    if (fault_state_ > 0.5) {
      watchdog_state_ = 1.0;
      return;
    }

    const double since_last = (now - last_frame_time_).seconds();
    const bool timed_out = since_last > timeout_sec_;

    if (timed_out) {
      watchdog_state_ = 1.0;
      if (!timeout_logged_) {
        RCLCPP_ERROR(logger_, "Limit switch 0x%03X timed out (%.3f s > %.3f s)",
                     can_id_, since_last, timeout_sec_);
        timeout_logged_ = true;
      }
    } else {
      watchdog_state_ = frame_received_ ? 0.0 : -1.0;
      if (timeout_logged_) {
        RCLCPP_WARN(logger_, "Limit switch 0x%03X recovered after timeout.",
                    can_id_);
        timeout_logged_ = false;
      }
    }
  }
  void export_state(double *&, double *&, double *&) override {}
  void export_command(double *&) override {}

  void export_named_states(
      std::vector<std::pair<std::string, double *>> &states) override {
    states.emplace_back(state_name_, &state_);
    states.emplace_back(fault_state_name_, &fault_state_);
    states.emplace_back(watchdog_state_name_, &watchdog_state_);
  }

private:
  void on_frame(const can_frame &frame) {
    const bool pressed = (frame.data[0] & 0x1) != 0;
    const double logical = pressed ? 1.0 : 0.0;
    const double mapped_state =
        active_high_ ? logical : (logical > 0.5 ? 0.0 : 1.0);

    state_ = mapped_state;

    const bool fault = (frame.data[1] & kFaultBit) != 0;
    fault_state_ = fault ? 1.0 : 0.0;

    if (fault) {
      if (!fault_reported_) {
        RCLCPP_WARN(logger_,
                    "Limit switch 0x%03X reported invalid contact state.",
                    can_id_);
        fault_reported_ = true;
      }
    } else if (fault_reported_) {
      RCLCPP_INFO(logger_, "Limit switch 0x%03X fault cleared.", can_id_);
      fault_reported_ = false;
    }

    if (ros_clock_) {
      last_frame_time_ = ros_clock_->now();
    }
    frame_received_ = true;
    watchdog_state_ = fault ? 1.0 : 0.0;
    timeout_logged_ = false;

    if (state_pub_ && can_publish()) {
      std_msgs::msg::Bool msg;
      msg.data = state_ > 0.5;
      safe_publish(state_pub_, msg);
    }

    if (fault_pub_ && can_publish()) {
      std_msgs::msg::Bool msg;
      msg.data = fault;
      safe_publish(fault_pub_, msg);
    }
  }

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
