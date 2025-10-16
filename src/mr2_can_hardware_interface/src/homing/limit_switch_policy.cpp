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
    backoff_speed_ = std::abs(parse_double(
        params, "backoff_speed",
        params.count("approach_speed") ? approach_speed_
                                       : kDefaultApproachSpeed));
    fine_speed_ =
        std::abs(parse_double(params, "fine_speed", kDefaultFineSpeed));
    backoff_distance_ =
        std::abs(parse_double(params, "backoff_distance", kDefaultBackoffDistance));
    timeout_ = std::max(1e-3, parse_double(params, "timeout", kDefaultTimeout));
    home_position_ = parse_double(params, "home_position", 0.0);

    sanitize_positive(approach_speed_, kDefaultApproachSpeed, "approach_speed");
    sanitize_positive(backoff_speed_, approach_speed_, "backoff_speed");
    sanitize_positive(fine_speed_, kDefaultFineSpeed, "fine_speed");
    sanitize_positive(backoff_distance_, kDefaultBackoffDistance, "backoff_distance");
    sanitize_positive(timeout_, kDefaultTimeout, "timeout");

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
    target_ = 0.0;
    phase_ = Phase::Idle;
    finished_ = false;
    error_ = false;
    backoff_remaining_ = 0.0;
    limit_sampled_ = false;
    limit_pressed_prev_ = false;
    initial_state_acquired_ =
        joint_.state && std::isfinite(*joint_.state);
    if (initial_state_acquired_) {
      target_ = *joint_.state;
      apply_target();
      transition_to(Phase::SearchFast, "begin homing");
    } else {
      if (joint_.command && std::isfinite(*joint_.command)) {
        target_ = *joint_.command;
      }
      if (node_) {
        RCLCPP_INFO(node_->get_logger(),
                    "LimitSwitchPolicy: waiting for initial joint state before homing");
      }
    }
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
        if (node_) {
          RCLCPP_ERROR(node_->get_logger(),
                       "LimitSwitchPolicy: %s", error_message_.c_str());
        }
        transition_to(Phase::Error, "watchdog timeout");
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
      if (node_) {
        RCLCPP_ERROR(node_->get_logger(),
                     "LimitSwitchPolicy: %s", error_message_.c_str());
      }
      transition_to(Phase::Error, "timeout exceeded");
      return;
    }

    const double dt = period.seconds();

    if (!initial_state_acquired_) {
      if (joint_.state && std::isfinite(*joint_.state)) {
        target_ = *joint_.state;
        apply_target();
        initial_state_acquired_ = true;
        start_time_ = now;
        transition_to(Phase::SearchFast,
                      "initial joint state acquired");
      } else {
        if (joint_.command && std::isfinite(*joint_.command)) {
          target_ = *joint_.command;
        }
      }
      return;
    }
    const double limit_val = limit_state_ ? *limit_state_ : 0.0;
    const bool sample_valid =
        !limit_watchdog_state_ || *limit_watchdog_state_ >= -0.5;

    if (!sample_valid) {
      start_time_ = now;
      return;
    }

    const bool limit_pressed = limit_val > kReleaseThreshold;

    if (!limit_sampled_) {
      limit_sampled_ = true;
      limit_pressed_prev_ = limit_pressed;
      if (limit_pressed) {
        backoff_remaining_ = backoff_distance_;
        transition_to(Phase::Backoff,
                      "initial limit engaged at start");
      }
    }

    switch (phase_) {
    case Phase::SearchFast:
      if (limit_pressed && !limit_pressed_prev_) {
        backoff_remaining_ = backoff_distance_;
        transition_to(Phase::Backoff,
                      "limit detected during fast approach");
        break;
      }
      integrate_target(dt, search_sign_ * approach_speed_);
      break;

    case Phase::Backoff:
      integrate_target(dt, -search_sign_ * backoff_speed_);
      backoff_remaining_ -= backoff_speed_ * dt;
      if (limit_val < kReleaseThreshold && backoff_remaining_ <= 0.0) {
        transition_to(Phase::ApproachSlow,
                      "completed backoff, re-approaching slowly");
      }
      break;

    case Phase::ApproachSlow:
      if (limit_pressed && !limit_pressed_prev_) {
        transition_to(Phase::Capture,
                      "limit detected during fine approach");
        finalize_offsets();
        finished_ = true;
        transition_to(Phase::Done, "homing sequence complete");
        break;
      }
      integrate_target(dt, search_sign_ * fine_speed_);
      break;

    case Phase::Capture:
      // Capture handled in ApproachSlow when limit re-engages.
      break;

    case Phase::Done:
      finished_ = true;
      break;

    case Phase::Error:
      error_ = true;
      break;
    }

    apply_target();
    limit_pressed_prev_ = limit_pressed;
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
    limit_sampled_ = false;
    limit_pressed_prev_ = false;
    initial_state_acquired_ = false;
  }

private:
  enum class Phase { Idle, SearchFast, Backoff, ApproachSlow, Capture, Done, Error };

  static const char *phase_name(Phase phase) {
    switch (phase) {
    case Phase::Idle:
      return "idle";
    case Phase::SearchFast:
      return "search_fast";
    case Phase::Backoff:
      return "backoff";
    case Phase::ApproachSlow:
      return "approach_slow";
    case Phase::Capture:
      return "capture";
    case Phase::Done:
      return "done";
    case Phase::Error:
      return "error";
    }
    return "unknown";
  }

  void transition_to(Phase new_phase, const char *reason) {
    if (phase_ == new_phase) {
      return;
    }
    Phase old_phase = phase_;
    phase_ = new_phase;
    if (node_) {
      if (new_phase == Phase::Error) {
        if (reason) {
          RCLCPP_ERROR(node_->get_logger(),
                       "LimitSwitchPolicy: %s -> %s (%s)",
                       phase_name(old_phase), phase_name(new_phase), reason);
        } else {
          RCLCPP_ERROR(node_->get_logger(),
                       "LimitSwitchPolicy: %s -> %s",
                       phase_name(old_phase), phase_name(new_phase));
        }
      } else {
        if (reason) {
          RCLCPP_INFO(node_->get_logger(),
                      "LimitSwitchPolicy: %s -> %s (%s)",
                      phase_name(old_phase), phase_name(new_phase), reason);
        } else {
          RCLCPP_INFO(node_->get_logger(),
                      "LimitSwitchPolicy: %s -> %s",
                      phase_name(old_phase), phase_name(new_phase));
        }
      }
    }
  }

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
  double backoff_speed_{kDefaultApproachSpeed};
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
  bool limit_sampled_{false};
  bool limit_pressed_prev_{false};
  bool initial_state_acquired_{false};

  void sanitize_positive(double &value, double fallback,
                         const char *param_name) {
    if (!std::isfinite(value) || value <= 0.0) {
      value = fallback;
      if (node_) {
        RCLCPP_WARN(node_->get_logger(),
                    "LimitSwitchPolicy: parameter '%s' invalid, using %.3f",
                    param_name, fallback);
      }
    }
  }
};

} // namespace mr2_can_hardware_interface

PLUGINLIB_EXPORT_CLASS(mr2_can_hardware_interface::LimitSwitchPolicy,
                       mr2_can_hardware_interface::HomingPolicy)
