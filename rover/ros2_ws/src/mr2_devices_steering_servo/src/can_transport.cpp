#include "can_transport.h"

#include "mr2_can_bus_core/can_bus_registry.hpp"

bool CanTransport::init(const std::string &iface, int bitrate) {
  bus_ = CanBusRegistry::get(iface, bitrate);
  if (!bus_) {
    return false;
  }

  bus_->register_listener(0, 0, [this](const can_frame &frame) {
    on_frame(frame);
  });
  return true;
}

bool CanTransport::pop_frame(can_frame &frame) {
  std::lock_guard<std::mutex> lock(rx_mutex_);
  if (rx_queue_.empty()) {
    return false;
  }
  frame = rx_queue_.front();
  rx_queue_.pop();
  return true;
}

void CanTransport::send_frame(const can_frame &frame) {
  if (!bus_) {
    return;
  }
  bus_->enqueue_tx(frame);
}

void CanTransport::on_frame(const can_frame &frame) {
  std::lock_guard<std::mutex> lock(rx_mutex_);
  if (rx_queue_.size() >= max_queue_depth_) {
    rx_queue_.pop();
  }
  rx_queue_.push(frame);
}
