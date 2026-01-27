#include <chrono>
#include <thread>

#include "board_config.h"
#include "can_service.h"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto logger = rclcpp::get_logger("steering_fw");

  CanService::Config config;
  config.can_iface = board_config::CAN_INTERFACE;
  config.bitrate = static_cast<int>(board_config::CAN_BITRATE);
  config.node_id = board_config::CAN_NODE_ID;
  config.pos_tx_ms = board_config::CAN_POS_TX_MS;
  config.timeout_ms = board_config::CAN_TIMEOUT_MS;
  config.rx_recover_ms = board_config::RX_RECOVER_MS;
  config.min_centi_deg = board_config::STEERING_MIN_CENTI_DEG;
  config.max_centi_deg = board_config::STEERING_MAX_CENTI_DEG;

  CanService service(logger);
  if (!service.init(config)) {
    rclcpp::shutdown();
    return 1;
  }

  constexpr int kPollSleepMs = 1;
  while (rclcpp::ok()) {
    service.poll();
    std::this_thread::sleep_for(std::chrono::milliseconds(kPollSleepMs));
  }

  rclcpp::shutdown();
  return 0;
}
