#include "can_service.h"

#include <algorithm>

#include "rclcpp/logging.hpp"

void SteeringApi::set_target_centi_deg(int16_t target) {
  target_centi_deg_ = clamp(target);
}

void SteeringApi::set_current_centi_deg(int16_t current) {
  current_centi_deg_ = clamp(current);
}

int16_t SteeringApi::current_centi_deg() const { return current_centi_deg_; }

int16_t SteeringApi::target_centi_deg() const { return target_centi_deg_; }

void SteeringApi::set_limits(int16_t min_centi_deg, int16_t max_centi_deg) {
  if (min_centi_deg > max_centi_deg) {
    min_centi_deg = max_centi_deg;
  }
  min_centi_deg_ = min_centi_deg;
  max_centi_deg_ = max_centi_deg;
  current_centi_deg_ = clamp(current_centi_deg_);
  target_centi_deg_ = clamp(target_centi_deg_);
}

void SteeringApi::step() {
  current_centi_deg_ = clamp(target_centi_deg_);
}

int16_t SteeringApi::clamp(int16_t value) const {
  return static_cast<int16_t>(
      std::min<int32_t>(max_centi_deg_,
                        std::max<int32_t>(min_centi_deg_, value)));
}

CanService::CanService(rclcpp::Logger logger) : logger_(logger) {}

bool CanService::init(const Config &config) {
  config_ = config;

  if (!transport_.init(config_.can_iface, config_.bitrate)) {
    RCLCPP_ERROR(logger_, "Failed to initialize CAN transport on '%s'",
                 config_.can_iface.c_str());
    return false;
  }

  steering_.set_limits(config_.min_centi_deg, config_.max_centi_deg);

  const auto now = Clock::now();
  last_command_time_ = now;
  last_tx_time_ = now;
  target_command_ = steering_.current_centi_deg();
  pending_command_ = target_command_;
  return true;
}

void CanService::poll() {
  const auto now = Clock::now();

  can_frame frame{};
  while (transport_.pop_frame(frame)) {
    int16_t centi_deg = 0;
    if (CanProtocol::decode_steering_command(frame, config_.node_id,
                                             centi_deg)) {
      handle_command(centi_deg, now);
    }
  }

  check_timeout(now);

  steering_.set_target_centi_deg(target_command_);
  steering_.step();

  maybe_send_position(now);
}

void CanService::handle_command(int16_t centi_deg,
                                const Clock::time_point &now) {
  last_command_time_ = now;
  pending_command_ = centi_deg;

  if (timeout_active_) {
    if (!recovery_active_) {
      recovery_active_ = true;
      recovery_start_ = now;
    }
    return;
  }

  target_command_ = centi_deg;
}

void CanService::check_timeout(const Clock::time_point &now) {
  if (!timeout_active_) {
    const auto elapsed_ms =
        std::chrono::duration_cast<std::chrono::milliseconds>(now -
                                                              last_command_time_)
            .count();
    if (elapsed_ms > static_cast<int64_t>(config_.timeout_ms)) {
      timeout_active_ = true;
      recovery_active_ = false;
      target_command_ = steering_.current_centi_deg();
      RCLCPP_WARN(logger_,
                  "Steering command timeout; holding current output.");
    }
    return;
  }

  if (!recovery_active_) {
    return;
  }

  const auto since_last_rx =
      std::chrono::duration_cast<std::chrono::milliseconds>(now -
                                                            last_command_time_)
          .count();
  if (since_last_rx > static_cast<int64_t>(config_.rx_recover_ms)) {
    recovery_active_ = false;
    return;
  }

  const auto recovery_ms =
      std::chrono::duration_cast<std::chrono::milliseconds>(now -
                                                            recovery_start_)
          .count();
  if (recovery_ms >= static_cast<int64_t>(config_.rx_recover_ms)) {
    timeout_active_ = false;
    recovery_active_ = false;
    target_command_ = pending_command_;
    RCLCPP_INFO(logger_, "Steering command RX recovered.");
  }
}

void CanService::maybe_send_position(const Clock::time_point &now) {
  if (config_.pos_tx_ms == 0) {
    return;
  }

  const auto elapsed_ms =
      std::chrono::duration_cast<std::chrono::milliseconds>(now - last_tx_time_)
          .count();
  if (elapsed_ms < static_cast<int64_t>(config_.pos_tx_ms)) {
    return;
  }

  last_tx_time_ = now;
  const auto current = steering_.current_centi_deg();
  const auto frame =
      CanProtocol::encode_steering_position(config_.node_id, current);
  transport_.send_frame(frame);
}
