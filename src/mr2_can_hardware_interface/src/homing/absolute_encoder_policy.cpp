/*
 * AbsoluteEncoderPolicy
 *
 * Homing policy that trusts an absolute encoder reading to determine the
 * joint offset. The plugin expects the following ROS parameters (prefixed
 * with `homing_` in the URDF/ros2_control description):
 *
 *   device_plugin (string, required)
 *     Name of the CAN device plugin that provides the encoder reading.
 *   device_* (various, forwarded)
 *     Any parameters prefixed with `homing_device_` are forwarded (prefix
 *     stripped) to the encoder device. Common ones are:
 *       - device_can_iface
 *       - device_can_id
 *       - device_ticks_per_rev
 *       - device_state_name (defaults to <joint>/absolute_encoder)
 *       - device_direction
 *   home_offset (double, optional, default 0.0)
 *     Additional offset in radians applied to the encoder angle before
 *     computing the joint offset.
 *   encoder_direction (double, optional, default 1.0)
 *     Multiplier that allows correcting for an inverted encoder (+1 or -1).
 */

#include "mr2_can_hardware_interface/homing_policy.hpp"

#include "mr2_devices_sensors/absolute_encoder_device.hpp"
#include "mr2_devices_sensors/homing_sensors.hpp"

#include "pluginlib/class_list_macros.hpp"

#include "rclcpp/logging.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <memory>
#include <unordered_map>

namespace mr2_can_hardware_interface {

class AbsoluteEncoderPolicy : public HomingPolicy {
public:
  void configure(const rclcpp::Node::SharedPtr &node,
                 const JointHandle &joint,
                 const ParamMap &params) override {
    node_ = node;
    joint_ = joint;

    // Split parameters into device_* (forwarded to device) and policy params.
    std::unordered_map<std::string, std::string> device_params;
    for (const auto &param : params) {
      if (param.first.rfind("device_", 0) == 0) {
        device_params.emplace(param.first.substr(std::string("device_").size()),
                              param.second);
      }
    }

    const std::string state_name =
        device_params.count("state_name")
            ? device_params["state_name"]
            : joint.name + "/absolute_encoder";
    device_params.emplace("state_name", state_name);

    // Build a minimal ComponentInfo for the device.
    hardware_interface::ComponentInfo component;
    component.name =
        device_params.count("name") ? device_params["name"] : state_name;
    component.type = "sensor";
    component.parameters = device_params;

    auto device = std::make_shared<mr2_devices_sensors::AbsoluteEncoderDevice>();
    try {
      device->configure(component, node_.get());
    } catch (const std::exception &ex) {
      error_message_ = std::string("Failed to configure absolute encoder: ") +
                       ex.what();
      error_ = true;
      return;
    }

    device_ = device;
    encoder_state_ = device_->angle_ptr();
    encoder_watchdog_state_ = device_->watchdog_ptr();
    if (!encoder_state_) {
      error_message_ = "Absolute encoder state pointer missing from device.";
      error_ = true;
      return;
    }

    // Helper used for optional numeric parameters.
    auto parse_double = [&](const std::string &key, double &target) -> bool {
      const auto it = params.find(key);
      if (it == params.end()) {
        return true;
      }
      try {
        target = std::stod(it->second);
      } catch (const std::exception &) {
        error_message_ = "Invalid numeric value for " + key + ": " + it->second;
        error_ = true;
        return false;
      }
      return true;
    };

    if (!parse_double("home_offset", home_offset_)) {
      return;
    }
    if (!parse_double("encoder_direction", encoder_direction_)) {
      return;
    }
    if (!std::isfinite(encoder_direction_) ||
        std::abs(encoder_direction_) < 1e-6) {
      error_message_ = "encoder_direction must be non-zero and finite.";
      error_ = true;
      return;
    }
  }

  void begin(const rclcpp::Time &now) override {
    (void)now;
    finished_ = false;
    error_ = false;
    computed_ = false;
    error_message_.clear();
  }

  void update(const rclcpp::Time &, const rclcpp::Duration &) override {
    if (error_ || finished_) {
      return;
    }

    if (encoder_watchdog_state_) {
      const double watchdog = *encoder_watchdog_state_;
      if (watchdog > 0.5) {
        error_ = true;
        error_message_ = "Absolute encoder watchdog reported timeout.";
        return;
      }
      if (watchdog < -0.5) {
        return;
      }
    }

    if (!encoder_state_ || !joint_.state || !joint_.offset) {
      error_ = true;
      error_message_ = "Absolute encoder pointers not initialised.";
      return;
    }

    if (!computed_) {
      const double absolute_angle =
          encoder_direction_ * (*encoder_state_) + home_offset_;
      const double joint_angle = *joint_.state;
      *joint_.command = joint_angle;
      joint_offset_ = joint_angle - absolute_angle;
      computed_ = true;
      if (node_) {
        RCLCPP_INFO(node_->get_logger(),
                    "Homed joint '%s' via absolute encoder: offset=%.6f rad "
                    "(encoder=%.6f rad, joint=%.6f rad)",
                    joint_.name.c_str(), joint_offset_, absolute_angle,
                    joint_angle);
      }
    }

    finished_ = true;
  }

  bool is_finished() const override { return finished_; }
  bool has_error() const override { return error_; }
  std::string error_message() const override { return error_message_; }

  void finalize(const rclcpp::Time &) override {
    *joint_.offset += joint_offset_;
    *joint_.command -= joint_offset_;
  }

  void reset() override {
    finished_ = false;
    error_ = false;
    computed_ = false;
    error_message_.clear();
  }

  std::shared_ptr<CanDevice> homing_device() const override {
    return std::static_pointer_cast<CanDevice>(device_);
  }

private:
  rclcpp::Node::SharedPtr node_;
  JointHandle joint_;
  std::shared_ptr<mr2_devices_sensors::AbsoluteEncoderDriver> device_;
  const double *encoder_state_{nullptr};
  const double *encoder_watchdog_state_{nullptr};
  double home_offset_{0.0};
  double encoder_direction_{1.0};

  bool finished_{false};
  bool computed_{false};
  bool error_{false};
  double joint_offset_{0.0};
  std::string error_message_;
};

} // namespace mr2_can_hardware_interface

PLUGINLIB_EXPORT_CLASS(mr2_can_hardware_interface::AbsoluteEncoderPolicy,
                       mr2_can_hardware_interface::HomingPolicy)
