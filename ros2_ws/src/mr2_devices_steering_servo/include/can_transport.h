#pragma once

#include <linux/can.h>
#include <memory>
#include <mutex>
#include <queue>
#include <string>

#include "mr2_can_bus_core/can_bus_manager.hpp"

class CanTransport {
public:
  bool init(const std::string &iface, int bitrate);
  bool pop_frame(can_frame &frame);
  void send_frame(const can_frame &frame);

private:
  void on_frame(const can_frame &frame);

  std::shared_ptr<CanBusManager> bus_;
  std::mutex rx_mutex_;
  std::queue<can_frame> rx_queue_;
  size_t max_queue_depth_{256};
};
