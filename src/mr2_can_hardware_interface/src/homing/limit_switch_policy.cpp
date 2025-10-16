#include "mr2_can_hardware_interface/homing_policy.hpp"

#include "pluginlib/class_list_macros.hpp"

#include "rclcpp/logging.hpp"

#include <algorithm>
#include <cmath>
#include <string>

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

class LimitSwitchPolicy : public HomingPolicy {
public:
  void configure(const rclcpp::Node::SharedPtr &node,
                 const std::vector<JointHandle> &joints,
                 const NamedStateMap &named_states,
                 const ParamMap &params) override {
    node_ = node;

    if (joints.size() != 1) {
      error_message_ = "LimitSwitchPolicy expects exactly one joint.";
      error_ = true;
      return;
    }
    joint_ = joints.front();

    const std::string limit_key = parse_string(params, "limit_state");
    if (limit_key.empty()) {
      error_message_ = "Parameter 'limit_state' must be provided.";
      error_ = true;
      return;
    }

    const auto limit_it = named_states.find(limit_key);
    if (limit_it == named_states.end()) {
      error_message_ = "Named state '" + limit_key + "' not found.";
      error_ = true;
      return;
    }
    limit_state_ = limit_it->second;

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

    approach_speed_ =
        std::abs(parse_double(params, "approach_speed", kDefaultApproachSpeed));
    fine_speed_ =
        std::abs(parse_double(params, "fine_speed", kDefaultFineSpeed));
    backoff_distance_ =
        std::abs(parse_double(params, "backoff_distance", kDefaultBackoffDistance));
    timeout_ = std::max(1e-3, parse_double(params, "timeout", kDefaultTimeout));
    home_position_ = parse_double(params, "home_position", 0.0);

    const std::string dir =
        parse_string(params, "search_direction", "negative");
    if (dir == "positive" || dir == "+1") {
      search_sign_ = 1.0;
    } else {
      search_sign_ = -1.0;
    }
  }

  void begin(const rclcpp::Time &now) override {
    start_time_ = now;
    target_ = joint_.state ? *joint_.state : 0.0;
    phase_ = Phase::SearchFast;
    finished_ = false;
    error_ = false;
    backoff_remaining_ = 0.0;
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

    if ((now - start_time_).seconds() > timeout_) {
      error_ = true;
      error_message_ = "Homing timeout exceeded.";
      phase_ = Phase::Error;
      return;
    }

    const double dt = period.seconds();
    const double limit_val = limit_state_ ? *limit_state_ : 0.0;

    switch (phase_) {
    case Phase::SearchFast:
      integrate_target(dt, search_sign_ * approach_speed_);
      if (limit_val > kReleaseThreshold) {
        phase_ = Phase::Backoff;
        backoff_remaining_ = backoff_distance_;
      }
      break;

    case Phase::Backoff:
      integrate_target(dt, -search_sign_ * approach_speed_);
      backoff_remaining_ -= std::abs(search_sign_ * approach_speed_) * dt;
      if (limit_val < kReleaseThreshold && backoff_remaining_ <= 0.0) {
        phase_ = Phase::ApproachSlow;
      }
      break;

    case Phase::ApproachSlow:
      integrate_target(dt, search_sign_ * fine_speed_);
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

    apply_target();
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
    error_message_.clear();
    phase_ = Phase::Idle;
    backoff_remaining_ = 0.0;
  }

private:
  enum class Phase { Idle, SearchFast, Backoff, ApproachSlow, Capture, Done, Error };

  void integrate_target(double dt, double velocity) {
    if (!joint_.command) {
      return;
    }
    target_ += velocity * dt;
  }

  void apply_target() {
    if (joint_.command) {
      *joint_.command = target_;
    }
  }

  void finalize_offsets() {
    if (!joint_.offset || !joint_.state) {
      error_ = true;
      error_message_ = "Joint state or offset pointer invalid during homing.";
      phase_ = Phase::Error;
      return;
    }
    const double raw_position = *joint_.state + *joint_.offset;
    *joint_.offset = raw_position - home_position_;
    target_ = home_position_;
    if (node_) {
      RCLCPP_INFO(
          node_->get_logger(),
          "Homed joint '%s': offset=%.6f rad (raw=%.6f rad, home=%.6f rad)",
          joint_.name.c_str(), *joint_.offset, raw_position, home_position_);
    }
  }

  rclcpp::Node::SharedPtr node_;
  JointHandle joint_;

  const double *limit_state_{nullptr};
  const double *limit_watchdog_state_{nullptr};

  double approach_speed_{kDefaultApproachSpeed};
  double fine_speed_{kDefaultFineSpeed};
  double backoff_distance_{kDefaultBackoffDistance};
  double backoff_remaining_{0.0};
  double search_sign_{-1.0};
  double timeout_{kDefaultTimeout};
  double home_position_{0.0};

  double target_{0.0};
  Phase phase_{Phase::Idle};
  bool finished_{false};
  bool error_{false};
  std::string error_message_;
  rclcpp::Time start_time_;
};

} // namespace mr2_can_hardware_interface

PLUGINLIB_EXPORT_CLASS(mr2_can_hardware_interface::LimitSwitchPolicy,
                       mr2_can_hardware_interface::HomingPolicy)
