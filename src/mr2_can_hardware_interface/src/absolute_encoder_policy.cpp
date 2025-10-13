#include "mr2_can_hardware_interface/homing_policy.hpp"

#include "pluginlib/class_list_macros.hpp"

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
      error_message_ =
          "Named state '" + state_it->second + "' not found for absolute encoder.";
      error_ = true;
      return;
    }

    encoder_state_ = ptr_it->second;

    auto offset_it = params.find("home_offset");
    if (offset_it != params.end()) {
      try {
        home_offset_ = std::stod(offset_it->second);
      } catch (const std::exception &) {
        error_message_ =
            "Invalid numeric value for home_offset: " + offset_it->second;
        error_ = true;
        return;
      }
    }
  }

  void begin(const rclcpp::Time &now) override {
    (void)now;
    finished_ = false;
    error_ = false;
    computed_ = false;
  }

  void update(const rclcpp::Time &, const rclcpp::Duration &) override {
    if (error_ || finished_) {
      return;
    }

    if (!encoder_state_ || !joint_.state || !joint_.offset) {
      error_ = true;
      error_message_ = "Absolute encoder pointers not initialised.";
      return;
    }

    if (!computed_) {
      const double absolute_angle = *encoder_state_ + home_offset_;
      const double joint_angle = *joint_.state;
      *joint_.offset = joint_angle - absolute_angle;
      computed_ = true;
    }

    finished_ = true;
  }

  bool is_finished() const override { return finished_; }
  bool has_error() const override { return error_; }
  std::string error_message() const override { return error_message_; }

  void finalize(const rclcpp::Time &) override {
    if (joint_.command && joint_.state) {
      *joint_.command = *joint_.state;
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
  double home_offset_{0.0};

  bool finished_{false};
  bool computed_{false};
  bool error_{false};
  std::string error_message_;
};

} // namespace mr2_can_hardware_interface

PLUGINLIB_EXPORT_CLASS(mr2_can_hardware_interface::AbsoluteEncoderPolicy,
                       mr2_can_hardware_interface::HomingPolicy)
