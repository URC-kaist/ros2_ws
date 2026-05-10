#include <cstdint>
#include <chrono>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>

#include <linux/can.h>

#include "mr2_can_bus_core/can_bus_registry.hpp"
#include "mr2_led/srv/set_led_mode.hpp"
#include "rclcpp/rclcpp.hpp"

namespace {
constexpr uint32_t kStdIdMask = 0x7FF;
constexpr auto kShutdownLedFlushDelay = std::chrono::milliseconds(50);
}

class LedCanNode : public rclcpp::Node {
public:
  LedCanNode() : rclcpp::Node("mr2_led") {
    can_iface_ = declare_parameter<std::string>("can_iface", "can0");
    const int can_id_param = declare_parameter<int>("can_id", 0x123);
    if (can_id_param < 0 || can_id_param > static_cast<int>(kStdIdMask)) {
      throw std::runtime_error("Invalid CAN ID: " + std::to_string(can_id_param));
    }
    can_id_ = static_cast<uint32_t>(can_id_param);

    bus_ = CanBusRegistry::get(can_iface_);
    if (!bus_) {
      throw std::runtime_error("Failed to acquire CAN bus on " + can_iface_);
    }

    service_ = create_service<mr2_led::srv::SetLedMode>(
        "set_led_mode",
        std::bind(&LedCanNode::handle_request_, this, std::placeholders::_1,
                  std::placeholders::_2, std::placeholders::_3));

    RCLCPP_INFO(get_logger(), "mr2_led ready on %s, StdID 0x%03X", 
                can_iface_.c_str(), can_id_);
  }

  ~LedCanNode() override { set_shutdown_led_mode_(); }

private:
  void send_mode_(uint8_t mode) {
    struct can_frame frame {};
    frame.can_id = can_id_ & kStdIdMask;
    frame.can_dlc = 1;
    frame.data[0] = mode;
    bus_->enqueue_tx(frame);
  }

  void set_shutdown_led_mode_() noexcept {
    try {
      if (!bus_) {
        return;
      }
      send_mode_(mr2_led::srv::SetLedMode::Request::MODE_SUCCESS);
      std::this_thread::sleep_for(kShutdownLedFlushDelay);
    } catch (const std::exception &ex) {
      RCLCPP_WARN(get_logger(), "Failed to set shutdown LED mode: %s",
                  ex.what());
    } catch (...) {
      RCLCPP_WARN(get_logger(), "Failed to set shutdown LED mode");
    }
  }

  void handle_request_(
      const std::shared_ptr<rmw_request_id_t> /*request_header*/,
      const std::shared_ptr<mr2_led::srv::SetLedMode::Request> request,
      std::shared_ptr<mr2_led::srv::SetLedMode::Response> response) {
    const auto mode = request->mode;
    if (mode > mr2_led::srv::SetLedMode::Request::MODE_SUCCESS) {
      response->success = false;
      response->message = "Invalid mode";
      RCLCPP_WARN(get_logger(), "Rejected mode %u", mode);
      return;
    }

    send_mode_(mode);

    response->success = true;
    response->message = "Sent";
  }

  std::string can_iface_;
  uint32_t can_id_{0};
  std::shared_ptr<CanBusManager> bus_;
  rclcpp::Service<mr2_led::srv::SetLedMode>::SharedPtr service_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<LedCanNode>();
    rclcpp::spin(node);
  } catch (const std::exception &ex) {
    RCLCPP_FATAL(rclcpp::get_logger("mr2_led"), "%s", ex.what());
  }
  rclcpp::shutdown();
  return 0;
}
