#include "mr2_can_hardware_interface/homing_policy.hpp"

#include "pluginlib/class_list_macros.hpp"

#include "rclcpp/logging.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>

namespace mr2_can_hardware_interface {

namespace {
constexpr double kDefaultApproachSpeed = 0.3;     // rad/s
constexpr double kDefaultFineSpeed = 0.05;        // rad/s
constexpr double kDefaultBackoffDistance = 0.15;  // rad
constexpr double kDefaultTimeout = 20.0;          // s
constexpr double kReleaseThreshold = 0.5;         // boolean latch threshold

double parse_double(const HomingPolicy::ParamMap &params,
                    const std::string &key, double def) {
  const auto it = params.find(key);
  if (it == params.end()) {
    return def;
  }
  try {
    return std::stod(it->second);
  } catch (const std::exception &) {
    return def;
  }
}

std::string parse_string(const HomingPolicy::ParamMap &params,
                         const std::string &key,
                         const std::string &def = "") {
  const auto it = params.find(key);
  return it == params.end() ? def : it->second;
}

} // namespace

class CoupledLimitEncoderPolicy : public HomingPolicy {
public:
  void configure(const rclcpp::Node::SharedPtr &node,
                 const std::vector<JointHandle> &joints,
                 const NamedStateMap &named_states,
                 const ParamMap &params) override {
    node_ = node;
    joints_ = joints;

    if (joints_.size() != 2) {
      error_message_ = "CoupledLimitEncoderPolicy expects exactly two joints.";
      error_ = true;
      return;
    }

    limit_state_key_ = parse_string(params, "limit_state");
    encoder_state_key_ = parse_string(params, "encoder_state");

    if (limit_state_key_.empty() || encoder_state_key_.empty()) {
      error_message_ =
          "Parameters 'limit_state' and 'encoder_state' must be provided.";
      error_ = true;
      return;
    }

    const auto limit_it = named_states.find(limit_state_key_);
    const auto encoder_it = named_states.find(encoder_state_key_);

    if (limit_it == named_states.end()) {
      error_message_ = "Named state '" + limit_state_key_ + "' not found.";
      error_ = true;
      return;
    }
    if (encoder_it == named_states.end()) {
      error_message_ = "Named state '" + encoder_state_key_ + "' not found.";
      error_ = true;
      return;
    }

    limit_state_ = limit_it->second;
    encoder_state_ = encoder_it->second;

    const std::string limit_error_key =
        parse_string(params, "limit_error_state");
    if (!limit_error_key.empty()) {
      const auto limit_error_it = named_states.find(limit_error_key);
      if (limit_error_it == named_states.end()) {
        error_message_ =
            "Named state '" + limit_error_key + "' not found for limit error.";
        error_ = true;
        return;
      }
      limit_error_state_ = limit_error_it->second;
    }

    const std::string limit_watchdog_key =
        parse_string(params, "limit_watchdog_state");
    if (!limit_watchdog_key.empty()) {
      const auto limit_watchdog_it = named_states.find(limit_watchdog_key);
      if (limit_watchdog_it == named_states.end()) {
        error_message_ =
            "Named state '" + limit_watchdog_key +
            "' not found for limit watchdog state.";
        error_ = true;
        return;
      }
      limit_watchdog_state_ = limit_watchdog_it->second;
    }

    const std::string encoder_error_key =
        parse_string(params, "encoder_error_state");
    if (!encoder_error_key.empty()) {
      const auto encoder_error_it = named_states.find(encoder_error_key);
      if (encoder_error_it == named_states.end()) {
        error_message_ =
            "Named state '" + encoder_error_key +
            "' not found for encoder error.";
        error_ = true;
        return;
      }
      encoder_error_state_ = encoder_error_it->second;
    }

    const std::string encoder_watchdog_key =
        parse_string(params, "encoder_watchdog_state");
    if (!encoder_watchdog_key.empty()) {
      const auto encoder_watchdog_it =
          named_states.find(encoder_watchdog_key);
      if (encoder_watchdog_it == named_states.end()) {
        error_message_ =
            "Named state '" + encoder_watchdog_key +
            "' not found for encoder watchdog state.";
        error_ = true;
        return;
      }
      encoder_watchdog_state_ = encoder_watchdog_it->second;
    }

    approach_speed_ =
        std::abs(parse_double(params, "approach_speed", kDefaultApproachSpeed));
    fine_speed_ =
        std::abs(parse_double(params, "fine_speed", kDefaultFineSpeed));
    backoff_distance_ =
        std::abs(parse_double(params, "backoff_distance", kDefaultBackoffDistance));
    timeout_ = std::max(
        1e-3, parse_double(params, "timeout", kDefaultTimeout));
    encoder_home_shift_ = parse_double(params, "encoder_home_shift", 0.0);

    const std::string dir =
        parse_string(params, "search_direction", "negative");
    search_sign_ = (dir == "positive" || dir == "+1" || dir == "positive")
                       ? 1.0
                       : -1.0;

  }

  void begin(const rclcpp::Time &now) override {
    start_time_ = now;
    for (size_t i = 0; i < joints_.size(); ++i) {
      targets_[i] = joints_[i].state ? *joints_[i].state : 0.0;
    }

    phase_ = Phase::SearchFast;
  }

  void update(const rclcpp::Time &now,
              const rclcpp::Duration &period) override {
    if (error_ || finished_) {
      return;
    }

    if (limit_watchdog_state_) {
      const double watchdog = *limit_watchdog_state_;
      if (watchdog > 0.5) {
        error_ = true;
        error_message_ = "Limit switch watchdog reported timeout.";
        phase_ = Phase::Error;
        return;
      }
      if (watchdog < -0.5) {
        start_time_ = now;
        return;
      }
    }

    if (limit_error_state_ && *limit_error_state_ > 0.5) {
      error_ = true;
      error_message_ = "Limit switch watchdog reported timeout.";
      phase_ = Phase::Error;
      return;
    }

    if (encoder_watchdog_state_) {
      const double watchdog = *encoder_watchdog_state_;
      if (watchdog > 0.5) {
        error_ = true;
        error_message_ = "Absolute encoder watchdog reported timeout.";
        phase_ = Phase::Error;
        return;
      }
      if (watchdog < -0.5) {
        start_time_ = now;
        return;
      }
    }

    if (encoder_error_state_ && *encoder_error_state_ > 0.5) {
      error_ = true;
      error_message_ = "Absolute encoder watchdog reported timeout.";
      phase_ = Phase::Error;
      return;
    }

    if ((now - start_time_).seconds() > timeout_) {
      error_ = true;
      error_message_ = "Homing timeout exceeded.";
      return;
    }

    const double dt = period.seconds();
    const double limit_val = limit_state_ ? *limit_state_ : 0.0;

    switch (phase_) {
    case Phase::SearchFast:
      integrate_targets(dt, search_sign_ * approach_speed_);
      if (limit_val > kReleaseThreshold) {
        phase_ = Phase::Backoff;
        backoff_remaining_ = backoff_distance_;
      }
      break;

    case Phase::Backoff:
      integrate_targets(dt, -search_sign_ * approach_speed_);
      backoff_remaining_ -= std::abs(search_sign_ * approach_speed_) * dt;
      if (limit_val < kReleaseThreshold && backoff_remaining_ <= 0.0) {
        phase_ = Phase::ApproachSlow;
      }
      break;

    case Phase::ApproachSlow:
      integrate_targets(dt, search_sign_ * fine_speed_);
      if (limit_val > kReleaseThreshold) {
        phase_ = Phase::Capture;
      }
      break;

    case Phase::Capture:
      finalize_offsets();
      finished_ = true;
      phase_ = Phase::Done;
      break;

    case Phase::Done:
      finished_ = true;
      break;

    case Phase::Error:
      error_ = true;
      break;
    }

    apply_targets();
  }

  bool is_finished() const override { return finished_; }

  bool has_error() const override { return error_; }

  std::string error_message() const override { return error_message_; }

  void finalize(const rclcpp::Time &) override {
    // Hold current positions to avoid sudden jump.
    for (auto &joint : joints_) {
      if (joint.command && joint.state) {
        *joint.command = *joint.state;
      }
    }
  }

  void reset() override {
    finished_ = false;
    error_ = false;
    error_message_.clear();
    phase_ = Phase::Idle;
    backoff_remaining_ = 0.0;
  }

private:
  enum class Phase { Idle, SearchFast, Backoff, ApproachSlow, Capture, Done, Error };

  void integrate_targets(double dt, double velocity) {
    for (auto &joint : joints_) {
      if (!joint.command || !joint.state) {
        continue;
      }
      double &target = select_target(joint.name);
      target += velocity * dt;
    }
  }

  void apply_targets() {
    for (size_t i = 0; i < joints_.size(); ++i) {
      if (!joints_[i].command)
        continue;
      *joints_[i].command = targets_[i];
    }
  }

  void finalize_offsets() {
    double joint0_offset = 0.0;
    double joint1_offset = 0.0;
    double joint0_position = 0.0;
    double joint1_position = 0.0;
    bool joint0_valid = false;
    bool joint1_valid = false;

    if (joints_[0].offset && joints_[0].state) {
      joint0_position = *joints_[0].state;
      *joints_[0].offset = joint0_position;
      joint0_offset = *joints_[0].offset;
      joint0_valid = true;
    }
    if (joints_[1].offset && joints_[1].state) {
      joint1_position = *joints_[1].state;
      const double encoder_angle =
          encoder_state_ ? *encoder_state_ : joint1_position;
      const double desired = encoder_angle + encoder_home_shift_;
      *joints_[1].offset = joint1_position - desired;
      joint1_offset = *joints_[1].offset;
      joint1_valid = true;
    }

    if (node_ && joint0_valid && joint1_valid) {
      const double encoder_angle =
          encoder_state_ ? *encoder_state_ : joint1_position;
      RCLCPP_INFO(node_->get_logger(),
                  "Homed coupled joints '%s'/'%s': offsets=(%.6f, %.6f) rad "
                  "(joint states=(%.6f, %.6f) rad, encoder=%.6f rad, shift=%.6f rad)",
                  joints_[0].name.c_str(), joints_[1].name.c_str(),
                  joint0_offset, joint1_offset, joint0_position, joint1_position,
                  encoder_angle, encoder_home_shift_);
    }
  }

  double &select_target(const std::string &joint_name) {
    if (joints_.empty())
      return targets_[0];
    if (joint_name == joints_[0].name) {
      return targets_[0];
    }
    if (joints_.size() > 1 && joint_name == joints_[1].name) {
      return targets_[1];
    }
    return targets_[0];
  }

  rclcpp::Node::SharedPtr node_;
  std::vector<JointHandle> joints_;

  const double *limit_state_{nullptr};
  const double *encoder_state_{nullptr};
  const double *limit_error_state_{nullptr};
  const double *encoder_error_state_{nullptr};
  const double *limit_watchdog_state_{nullptr};
  const double *encoder_watchdog_state_{nullptr};

  std::string limit_state_key_;
  std::string encoder_state_key_;

  double approach_speed_{kDefaultApproachSpeed};
  double fine_speed_{kDefaultFineSpeed};
  double backoff_distance_{kDefaultBackoffDistance};
  double backoff_remaining_{0.0};
  double search_sign_{-1.0};
  double encoder_home_shift_{0.0};
  double timeout_{kDefaultTimeout};

  Phase phase_{Phase::Idle};
  double targets_[2]{0.0, 0.0};
  bool finished_{false};
  bool error_{false};
  std::string error_message_;
  rclcpp::Time start_time_;
};

} // namespace mr2_can_hardware_interface

PLUGINLIB_EXPORT_CLASS(
    mr2_can_hardware_interface::CoupledLimitEncoderPolicy,
    mr2_can_hardware_interface::HomingPolicy)
