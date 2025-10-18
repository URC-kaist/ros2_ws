#include "mr2_can_hardware_interface/homing_policy.hpp"

#include "pluginlib/class_list_macros.hpp"

#include "rclcpp/logging.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <string>
#include <utility>
#include <vector>

namespace mr2_can_hardware_interface {

namespace {
constexpr double kDefaultApproachSpeed = 0.3;    // rad/s
constexpr double kDefaultFineSpeed = 0.05;       // rad/s
constexpr double kDefaultBackoffDistance = 0.15; // rad
constexpr double kDefaultMaxTravel = std::numeric_limits<double>::infinity();
constexpr double kReleaseThreshold = 0.5; // boolean latch threshold

std::string parse_string(const HomingPolicy::ParamMap &params,
                         const std::string &key, const std::string &def = "") {
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
    error_ = false;
    error_message_.clear();

    if (joints.size() != 1) {
      set_error("LimitSwitchPolicy expects exactly one joint.");
      return;
    }
    joint_ = joints.front();

    const std::string limit_key = parse_string(params, "limit_state");
    if (limit_key.empty()) {
      set_error("Parameter 'limit_state' must be provided.");
      return;
    }

    const auto limit_it = named_states.find(limit_key);
    if (limit_it == named_states.end()) {
      set_error("Named state '" + limit_key + "' not found.");
      return;
    }
    limit_state_ = limit_it->second;

    const std::string watchdog_key =
        parse_string(params, "limit_watchdog_state");
    if (!watchdog_key.empty()) {
      const auto watchdog_it = named_states.find(watchdog_key);
      if (watchdog_it == named_states.end()) {
        set_error("Named state '" + watchdog_key +
                  "' not found for limit watchdog state.");
        return;
      }
      limit_watchdog_state_ = watchdog_it->second;
    }

    auto get_double = [&](const std::string &key, double def) {
      const auto it = params.find(key);
      if (it == params.end()) {
        return def;
      }
      try {
        return std::stod(it->second);
      } catch (const std::exception &ex) {
        log_warn("failed to parse parameter '%s': %s (using %.3f)", key.c_str(),
                 ex.what(), def);
        return def;
      }
    };

    auto ensure_positive = [&](const std::string &name, double value,
                               double fallback) {
      if (!std::isfinite(value) || value <= 0.0) {
        log_warn("parameter '%s' invalid, using %.3f", name.c_str(), fallback);
        return fallback;
      }
      return value;
    };

    approach_speed_ = ensure_positive(
        "approach_speed",
        std::abs(get_double("approach_speed", kDefaultApproachSpeed)),
        kDefaultApproachSpeed);

    const double default_backoff = params.count("approach_speed")
                                       ? approach_speed_
                                       : kDefaultApproachSpeed;
    backoff_speed_ = ensure_positive(
        "backoff_speed", std::abs(get_double("backoff_speed", default_backoff)),
        default_backoff);

    fine_speed_ = ensure_positive(
        "fine_speed", std::abs(get_double("fine_speed", kDefaultFineSpeed)),
        kDefaultFineSpeed);

    backoff_distance_ = ensure_positive(
        "backoff_distance",
        std::abs(get_double("backoff_distance", kDefaultBackoffDistance)),
        kDefaultBackoffDistance);

    const double max_travel =
        get_double("max_search_travel_rad", kDefaultMaxTravel);
    if (std::isfinite(max_travel) && max_travel <= 0.0) {
      log_warn("parameter '%s' invalid, ignoring", "max_search_travel_rad");
      max_search_travel_ = kDefaultMaxTravel;
    } else {
      max_search_travel_ = max_travel;
    }

    home_position_ = get_double("home_position", 0.0);

    const std::string direction =
        parse_string(params, "search_direction", "negative");
    if (direction == "positive" || direction == "+1" || direction == "pos") {
      search_sign_ = 1.0;
    } else {
      search_sign_ = -1.0;
    }
  }

  void begin(const rclcpp::Time &now) override {
    target_ = joint_.command && std::isfinite(*joint_.command) ? *joint_.command
                                                               : 0.0;
    phase_ = Phase::AwaitState;
    finished_ = false;
    error_ = false;
    error_message_.clear();
    backoff_remaining_ = 0.0;
    have_limit_sample_ = false;
    limit_pressed_prev_ = false;
    origin_set_ = false;

    if (joint_.state && std::isfinite(*joint_.state)) {
      target_ = *joint_.state;
      apply_target();
      transition(Phase::FastSearch, "begin homing");
    } else {
      apply_target();
      log_info("waiting for initial joint state before homing");
    }
  }

  void update(const rclcpp::Time &now,
              const rclcpp::Duration &period) override {
    if (error_ || phase_ == Phase::Done) {
      return;
    }

    if (!handle_watchdog()) {
      return;
    }

    const double dt = period.seconds();

    if (phase_ == Phase::AwaitState) {
      if (joint_.state && std::isfinite(*joint_.state)) {
        target_ = *joint_.state;
        apply_target();
        transition(Phase::FastSearch, "initial joint state acquired");
      } else if (joint_.command && std::isfinite(*joint_.command)) {
        target_ = *joint_.command;
        apply_target();
      }
      return;
    }

    if (limit_watchdog_state_ && *limit_watchdog_state_ < -kReleaseThreshold) {
      return;
    }

    const double limit_val = limit_state_ ? *limit_state_ : 0.0;
    const bool limit_pressed = limit_val > kReleaseThreshold;

    if (!have_limit_sample_) {
      have_limit_sample_ = true;
      limit_pressed_prev_ = limit_pressed;
      if (limit_pressed) {
        backoff_remaining_ = backoff_distance_;
        transition(Phase::Backoff, "initial limit engaged");
      }
    }

    switch (phase_) {
    case Phase::FastSearch:
      if (!origin_set_) {
        search_origin_ = target_;
        origin_set_ = true;
      }
      if (limit_pressed && !limit_pressed_prev_) {
        backoff_remaining_ = backoff_distance_;
        transition(Phase::Backoff, "limit detected during fast approach");
        break;
      }
      integrate_target(dt, search_sign_ * approach_speed_);
      if (exceeded_travel_limit()) {
        break;
      }
      break;

    case Phase::Backoff:
      integrate_target(dt, -search_sign_ * backoff_speed_);
      backoff_remaining_ -= backoff_speed_ * dt;
      if (!limit_pressed && backoff_remaining_ <= 0.0) {
        transition(Phase::SlowSearch, "completed backoff");
      }
      break;

    case Phase::SlowSearch:
      if (limit_pressed && !limit_pressed_prev_) {
        finalize_offsets();
        if (!error_) {
          finished_ = true;
          target_ -= *joint_.offset;
          transition(Phase::Done, "homing sequence complete");
        }
        break;
      }
      integrate_target(dt, search_sign_ * fine_speed_);
      if (exceeded_travel_limit()) {
        break;
      }
      break;

    case Phase::Done:
    case Phase::Error:
    case Phase::Idle:
    case Phase::AwaitState:
      break;
    }

    apply_target();
    limit_pressed_prev_ = limit_pressed;
  }

  bool is_finished() const override { return finished_; }

  bool has_error() const override { return error_; }

  std::string error_message() const override { return error_message_; }

  void finalize(const rclcpp::Time &) override {}

  void reset() override {
    finished_ = false;
    error_ = false;
    error_message_.clear();
    phase_ = Phase::Idle;
    backoff_remaining_ = 0.0;
    have_limit_sample_ = false;
    limit_pressed_prev_ = false;
    origin_set_ = false;
  }

private:
  enum class Phase {
    Idle,
    AwaitState,
    FastSearch,
    Backoff,
    SlowSearch,
    Done,
    Error
  };

  bool handle_watchdog() {
    if (!limit_watchdog_state_) {
      return true;
    }
    const double watchdog = *limit_watchdog_state_;
    if (watchdog > kReleaseThreshold) {
      set_error("Limit switch watchdog reported timeout.");
      return false;
    }
    if (watchdog < -kReleaseThreshold) {
      return false;
    }
    return true;
  }

  bool exceeded_travel_limit() {
    if (!origin_set_) {
      return false;
    }
    if (!std::isfinite(max_search_travel_) || max_search_travel_ <= 0.0) {
      return false;
    }
    if (std::abs(target_ - search_origin_) <= max_search_travel_) {
      return false;
    }
    set_error("Homing search exceeded maximum travel range.");
    return true;
  }

  void integrate_target(double dt, double velocity) {
    if (joint_.command) {
      target_ += velocity * dt;
    }
  }

  void apply_target() {
    if (joint_.command) {
      *joint_.command = target_;
    }
  }

  void finalize_offsets() {
    if (!joint_.offset || !joint_.state) {
      set_error("Joint state or offset pointer invalid during homing.");
      return;
    }
    const double raw_position = *joint_.state + *joint_.offset;
    *joint_.offset = raw_position - home_position_;
    log_info("Homed joint '%s': offset=%.6f rad (raw=%.6f rad, home=%.6f rad)",
             joint_.name.c_str(), *joint_.offset, raw_position, home_position_);
  }

  void set_error(const std::string &message) {
    error_ = true;
    error_message_ = message;
    if (phase_ != Phase::Error) {
      transition(Phase::Error, message.c_str(), true);
    } else {
      log_error("%s", message.c_str());
    }
  }

  void transition(Phase new_phase, const char *reason,
                  bool force_error_log = false) {
    if (phase_ == new_phase) {
      if (force_error_log && reason) {
        log_error("%s", reason);
      }
      return;
    }
    Phase old_phase = phase_;
    phase_ = new_phase;
    if (!node_) {
      return;
    }
    const char *old_name = phase_name(old_phase);
    const char *new_name = phase_name(new_phase);
    if (new_phase == Phase::Error || force_error_log) {
      if (reason) {
        log_error("LimitSwitchPolicy: %s -> %s (%s)", old_name, new_name,
                  reason);
      } else {
        log_error("LimitSwitchPolicy: %s -> %s", old_name, new_name);
      }
    } else if (reason) {
      log_info("LimitSwitchPolicy: %s -> %s (%s)", old_name, new_name, reason);
    } else {
      log_info("LimitSwitchPolicy: %s -> %s", old_name, new_name);
    }
  }

  static const char *phase_name(Phase phase) {
    switch (phase) {
    case Phase::Idle:
      return "idle";
    case Phase::AwaitState:
      return "await_state";
    case Phase::FastSearch:
      return "fast_search";
    case Phase::Backoff:
      return "backoff";
    case Phase::SlowSearch:
      return "slow_search";
    case Phase::Done:
      return "done";
    case Phase::Error:
      return "error";
    }
    return "unknown";
  }

  template <typename... Args>
  void log_info(const char *fmt, Args &&...args) const {
    if (!node_) {
      return;
    }
    const auto message = format_message(fmt, std::forward<Args>(args)...);
    RCLCPP_INFO(node_->get_logger(), "%s", message.c_str());
  }

  template <typename... Args>
  void log_warn(const char *fmt, Args &&...args) const {
    if (!node_) {
      return;
    }
    const auto message = format_message(fmt, std::forward<Args>(args)...);
    RCLCPP_WARN(node_->get_logger(), "%s", message.c_str());
  }

  template <typename... Args>
  void log_error(const char *fmt, Args &&...args) const {
    if (!node_) {
      return;
    }
    const auto message = format_message(fmt, std::forward<Args>(args)...);
    RCLCPP_ERROR(node_->get_logger(), "%s", message.c_str());
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
  double max_search_travel_{kDefaultMaxTravel};
  double home_position_{0.0};

  double target_{0.0};
  Phase phase_{Phase::Idle};
  bool finished_{false};
  bool error_{false};
  std::string error_message_;
  bool have_limit_sample_{false};
  bool limit_pressed_prev_{false};
  double search_origin_{0.0};
  bool origin_set_{false};

  template <typename... Args>
  static std::string format_message(const char *fmt, Args &&...args) {
    if (!fmt) {
      return {};
    }
    const int size = std::snprintf(nullptr, 0, fmt, args...);
    if (size <= 0) {
      return std::string(fmt);
    }
    std::vector<char> buffer(static_cast<size_t>(size) + 1);
    std::snprintf(buffer.data(), buffer.size(), fmt, args...);
    return std::string(buffer.data());
  }
};

} // namespace mr2_can_hardware_interface

PLUGINLIB_EXPORT_CLASS(mr2_can_hardware_interface::LimitSwitchPolicy,
                       mr2_can_hardware_interface::HomingPolicy)
