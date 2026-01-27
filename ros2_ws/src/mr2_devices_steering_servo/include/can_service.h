#pragma once

#include <cstdint>
#include <chrono>
#include <string>

#include "can_protocol.h"
#include "can_transport.h"
#include "rclcpp/logger.hpp"

class SteeringApi {
public:
  void set_target_centi_deg(int16_t target);
  void set_current_centi_deg(int16_t current);
  int16_t current_centi_deg() const;
  int16_t target_centi_deg() const;
  void set_limits(int16_t min_centi_deg, int16_t max_centi_deg);
  void step();

private:
  int16_t clamp(int16_t value) const;

  int16_t current_centi_deg_{0};
  int16_t target_centi_deg_{0};
  int16_t min_centi_deg_{-32768};
  int16_t max_centi_deg_{32767};
};

class CanService {
public:
  struct Config {
    std::string can_iface;
    int bitrate{1'000'000};
    uint8_t node_id{0};
    uint32_t pos_tx_ms{0};
    uint32_t timeout_ms{100};
    uint32_t rx_recover_ms{50};
    int16_t min_centi_deg{-32768};
    int16_t max_centi_deg{32767};
  };

  explicit CanService(rclcpp::Logger logger);

  bool init(const Config &config);
  void poll();

private:
  using Clock = std::chrono::steady_clock;

  void handle_command(int16_t centi_deg, const Clock::time_point &now);
  void check_timeout(const Clock::time_point &now);
  void maybe_send_position(const Clock::time_point &now);

  rclcpp::Logger logger_;
  Config config_{};
  CanTransport transport_{};
  SteeringApi steering_{};

  bool timeout_active_{false};
  bool recovery_active_{false};
  int16_t pending_command_{0};
  int16_t target_command_{0};

  Clock::time_point last_command_time_{};
  Clock::time_point recovery_start_{};
  Clock::time_point last_tx_time_{};
};
