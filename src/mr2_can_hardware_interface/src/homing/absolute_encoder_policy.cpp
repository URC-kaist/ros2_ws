/*
 * AbsoluteEncoderPolicy
 *
 * Homing policy that trusts an absolute encoder reading to determine the
 * joint offset. The plugin expects the following ROS parameters (prefixed
 * with `homing_` in the URDF/ros2_control description):
 *
 *   encoder_state (string, required)
 *     Name of the shared state exported by the absolute encoder device.
 *   encoder_watchdog_state (string, optional)
 *     Name of the watchdog state to monitor encoder health.
 *   home_offset (double, optional, default 0.0)
 *     Additional offset in radians applied to the encoder angle before
 *     computing the joint offset.
 *   home_position (double, optional, default 0.0)
 *     Target joint angle in radians commanded once homing completes.
 *   encoder_direction (double, optional, default 1.0)
 *     Multiplier that allows correcting for an inverted encoder (+1 or -1).
 */

#include "mr2_can_hardware_interface/homing_policy.hpp"

#include "pluginlib/class_list_macros.hpp"

#include "rclcpp/logging.hpp"

#include <cmath>
#include <stdexcept>

namespace mr2_can_hardware_interface {

class AbsoluteEncoderPolicy : public HomingPolicy {
public:
  void configure(const rclcpp::Node::SharedPtr &node,
                 const std::vector<JointHandle> &joints,
                 const NamedStateMap &named_states,
                 const ParamMap &params) override {
    node_ = node;
    if (joints.size() != 1) {
      error_message_ = "AbsoluteEncoderPolicy expects exactly one joint.";
      error_ = true;
      return;
    }
    joint_ = joints.front();

    const auto state_it = params.find("encoder_state");
    if (state_it == params.end()) {
      error_message_ =
          "AbsoluteEncoderPolicy requires 'encoder_state' parameter.";
      error_ = true;
      return;
    }

    const auto ptr_it = named_states.find(state_it->second);
    if (ptr_it == named_states.end()) {
      error_message_ = "Named state '" + state_it->second +
                       "' not found for absolute encoder.";
      error_ = true;
      return;
    }

    encoder_state_ = ptr_it->second;

    const auto watchdog_state_it = params.find("encoder_watchdog_state");
    if (watchdog_state_it != params.end()) {
      const auto watchdog_ptr_it = named_states.find(watchdog_state_it->second);
      if (watchdog_ptr_it == named_states.end()) {
        error_message_ = "Named state '" + watchdog_state_it->second +
                         "' not found for absolute encoder watchdog.";
        error_ = true;
        return;
      }
      encoder_watchdog_state_ = watchdog_ptr_it->second;
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
    if (!parse_double("home_position", home_position_)) {
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
      *joint_.offset = joint_angle - absolute_angle;
      computed_ = true;
      // Command the joint directly to the configured home position.
      if (joint_.command) {
        *joint_.command = home_position_;
      }
      if (node_) {
        RCLCPP_INFO(node_->get_logger(),
                    "Homed joint '%s' via absolute encoder: offset=%.6f rad "
                    "(encoder=%.6f rad, joint=%.6f rad, target=%.6f rad)",
                    joint_.name.c_str(), *joint_.offset, absolute_angle,
                    joint_angle, home_position_);
      }
    }

    finished_ = true;
  }

  bool is_finished() const override { return finished_; }
  bool has_error() const override { return error_; }
  std::string error_message() const override { return error_message_; }

  void finalize(const rclcpp::Time &) override {
    if (joint_.command) {
      *joint_.command = home_position_;
    }
  }

  void reset() override {
    finished_ = false;
    error_ = false;
    computed_ = false;
    error_message_.clear();
  }

private:
  rclcpp::Node::SharedPtr node_;
  JointHandle joint_;
  const double *encoder_state_{nullptr};
  const double *encoder_watchdog_state_{nullptr};
  double home_offset_{0.0};
  double home_position_{0.0};
  double encoder_direction_{1.0};

  bool finished_{false};
  bool computed_{false};
  bool error_{false};
  std::string error_message_;
};

} // namespace mr2_can_hardware_interface

PLUGINLIB_EXPORT_CLASS(mr2_can_hardware_interface::AbsoluteEncoderPolicy,
                       mr2_can_hardware_interface::HomingPolicy)
