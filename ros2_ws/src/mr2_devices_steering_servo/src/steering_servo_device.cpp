#include "can_protocol.h"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "mr2_can_bus_core/can_bus_registry.hpp"
#include "mr2_can_bus_core/can_device.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/logging.hpp"

#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>

namespace mr2_devices_steering_servo {

class SteeringServoDevice : public CanDevice {
public:
  void configure(const hardware_interface::ComponentInfo &info,
                 rclcpp::Node *node) override {
    node_ = node;
    logger_ = node_ ? node_->get_logger()
                   : rclcpp::get_logger("steering_servo_device");

    const auto iface_it = info.parameters.find("can_iface");
    if (iface_it == info.parameters.end()) {
      throw std::runtime_error("Missing can_iface parameter");
    }
    iface_ = iface_it->second;

    const auto node_id_it = info.parameters.find("node_id");
    if (node_id_it == info.parameters.end()) {
      throw std::runtime_error("Missing node_id parameter");
    }
    node_id_ = static_cast<uint8_t>(std::stoi(node_id_it->second));

    int bitrate = 1'000'000;
    const auto bitrate_it = info.parameters.find("can_bitrate");
    if (bitrate_it != info.parameters.end()) {
      bitrate = std::stoi(bitrate_it->second);
    }

    bus_ = CanBusRegistry::get(iface_, bitrate);
    if (!bus_) {
      throw std::runtime_error("Cannot open CAN bus");
    }

    const uint32_t position_id =
        CanProtocol::steering_position_id(node_id_);
    add_filter(bus_, position_id, 0x7FF,
               [this](const can_frame &f) { on_position(f); });
  }

  void process(const rclcpp::Time &) override {
    double desired = desired_command_rad_;
    if (std::isnan(desired)) {
      if (std::isnan(hold_position_rad_)) {
        hold_position_rad_ = position_rad_;
      }
      desired = hold_position_rad_;
    } else {
      hold_position_rad_ = std::numeric_limits<double>::quiet_NaN();
    }

    if (std::isnan(desired)) {
      return;
    }

    const int16_t centi_deg = rad_to_centi_deg(desired);
    const auto frame = CanProtocol::encode_steering_command(node_id_, centi_deg);
    send(frame, bus_);
  }

  void export_state(double *&position, double *&velocity,
                    double *&effort) override {
    // Ensure we never expose NaN to the hardware interface / TF chain.
    if (!std::isfinite(position_rad_)) {
      position_rad_ = 0.0;
    }
    position = &position_rad_;
    velocity = nullptr;
    effort = nullptr;
  }

  void export_command(double *&command) override {
    command = &desired_command_rad_;
  }

private:
  static int16_t rad_to_centi_deg(double rad) {
    constexpr double kRadToDeg = 180.0 / M_PI;
    const double deg = rad * kRadToDeg;
    const long centi = std::lround(deg * 100.0);
    if (centi > std::numeric_limits<int16_t>::max()) {
      return std::numeric_limits<int16_t>::max();
    }
    if (centi < std::numeric_limits<int16_t>::min()) {
      return std::numeric_limits<int16_t>::min();
    }
    return static_cast<int16_t>(centi);
  }

  static double centi_deg_to_rad(int16_t centi_deg) {
    constexpr double kDegToRad = M_PI / 180.0;
    return (static_cast<double>(centi_deg) * 0.01) * kDegToRad;
  }

  void on_position(const can_frame &frame) {
    int16_t centi_deg = 0;
    if (!CanProtocol::decode_steering_position(frame, node_id_, centi_deg)) {
      return;
    }
    position_rad_ = centi_deg_to_rad(centi_deg);
  }

  std::shared_ptr<CanBusManager> bus_;
  rclcpp::Node *node_{nullptr};
  rclcpp::Logger logger_{rclcpp::get_logger("steering_servo_device")};
  std::string iface_;
  uint8_t node_id_{0};

  // Start at a neutral angle so robot_state_publisher doesn't emit invalid TFs
  // before the first feedback frame arrives from the servo.
  double position_rad_{0.0};
  double desired_command_rad_{std::numeric_limits<double>::quiet_NaN()};
  double hold_position_rad_{std::numeric_limits<double>::quiet_NaN()};
};

} // namespace mr2_devices_steering_servo

PLUGINLIB_EXPORT_CLASS(mr2_devices_steering_servo::SteeringServoDevice,
                       CanDevice)
