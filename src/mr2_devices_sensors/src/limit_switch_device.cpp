#include "mr2_can_bus_core/can_device.hpp"

#include "pluginlib/class_list_macros.hpp"

#include <cstdint>
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
    auto counter_it = info.parameters.find("counter_name");
    if (counter_it != info.parameters.end()) {
      counter_name_ = counter_it->second;
    }
    auto timestamp_it = info.parameters.find("timestamp_name");
    if (timestamp_it != info.parameters.end()) {
      timestamp_name_ = timestamp_it->second;
    }
    auto edge_it = info.parameters.find("edge_name");
    if (edge_it != info.parameters.end()) {
      edge_name_ = edge_it->second;
    }

    bus_ = CanBusRegistry::get(iface_, bitrate_);
    if (!bus_) {
      throw std::runtime_error("Cannot open CAN bus: " + iface_);
    }

    add_filter(bus_, can_id_, kStdIdMask,
               [this](const can_frame &frame) { on_frame(frame); });
  }

  void process(const rclcpp::Time &) override {}
  void export_state(std::vector<double *> &, std::vector<double *> &,
                    std::vector<double *> &) override {}
  void export_command(std::vector<double *> &) override {}

  void export_named_states(
      std::vector<std::pair<std::string, double *>> &states) override {
    states.emplace_back(state_name_, &state_);
    if (!counter_name_.empty()) {
      states.emplace_back(counter_name_, &counter_);
    }
    if (!timestamp_name_.empty()) {
      states.emplace_back(timestamp_name_, &timestamp_ms_);
    }
    if (!edge_name_.empty()) {
      states.emplace_back(edge_name_, &edge_);
    }
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
    state_ = active_high_ ? logical : (logical > 0.5 ? 0.0 : 1.0);

    const uint16_t counter = (static_cast<uint16_t>(frame.data[2]) << 8) |
                             static_cast<uint16_t>(frame.data[3]);
    counter_ = static_cast<double>(counter);

    const uint32_t timestamp = (static_cast<uint32_t>(frame.data[4]) << 24) |
                               (static_cast<uint32_t>(frame.data[5]) << 16) |
                               (static_cast<uint32_t>(frame.data[6]) << 8) |
                               static_cast<uint32_t>(frame.data[7]);
    timestamp_ms_ = static_cast<double>(timestamp);

    if (!edge_name_.empty()) {
      if (frame.data[1] & 0x1) {
        edge_ = 1.0;
      } else if (frame.data[1] & 0x2) {
        edge_ = -1.0;
      } else {
        edge_ = 0.0;
      }
    }
  }

  rclcpp::Node *node_{nullptr};
  std::shared_ptr<CanBusManager> bus_;
  std::string iface_;
  int bitrate_{1'000'000};
  uint32_t can_id_{0};
  bool active_high_{true};

  std::string state_name_;
  std::string counter_name_;
  std::string timestamp_name_;
  std::string edge_name_;

  double state_{0.0};
  double counter_{0.0};
  double timestamp_ms_{0.0};
  double edge_{0.0};
};

} // namespace mr2_devices_sensors

PLUGINLIB_EXPORT_CLASS(mr2_devices_sensors::LimitSwitchDevice, CanDevice)
