#include "mr2_can_bus_core/can_device.hpp"

#include "pluginlib/class_list_macros.hpp"

#include <cmath>
#include <cstdint>
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
    states.emplace_back(state_name_, &angle_rad_);
    if (!raw_name_.empty()) {
      states.emplace_back(raw_name_, &raw_counts_);
    }
    if (!flags_name_.empty()) {
      states.emplace_back(flags_name_, &flags_);
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
    const int32_t raw_q24 = unpack_s24_be(frame);
    const double counts = static_cast<double>(raw_q24) / static_cast<double>(1 << 12);
    raw_counts_ = counts;

    const double revolutions = counts / ticks_per_rev_;
    angle_rad_ = direction_ * revolutions * 2.0 * M_PI - zero_offset_rad_;

    if (!flags_name_.empty()) {
      flags_ = static_cast<double>(frame.data[3]);
    }
  }

  rclcpp::Node *node_{nullptr};
  std::shared_ptr<CanBusManager> bus_;
  std::string iface_;
  int bitrate_{1'000'000};
  uint32_t can_id_{0};
  double ticks_per_rev_{4096.0};
  double direction_{1.0};
  double zero_offset_rad_{0.0};

  std::string state_name_;
  std::string raw_name_;
  std::string flags_name_;

  double angle_rad_{0.0};
  double raw_counts_{0.0};
  double flags_{0.0};
};

} // namespace mr2_devices_sensors

PLUGINLIB_EXPORT_CLASS(mr2_devices_sensors::AbsoluteEncoderDevice, CanDevice)
