#include <chrono>
#include <cstdint>
#include <stdexcept>
#include <string>

#include <linux/can.h>

#include "rclcpp/rclcpp.hpp"

#include "mr2_can_bus_core/can_bus_manager.hpp"
#include "mr2_can_bus_core/can_bus_registry.hpp"

using namespace std::chrono_literals;

class MockLimitSwitchNode : public rclcpp::Node {
public:
  MockLimitSwitchNode() : rclcpp::Node("mock_limit_switch") {
    can_iface_ = declare_parameter<std::string>("can_iface", "vcan0");
    can_id_ = declare_parameter<int>("can_id", 0x181);
    update_rate_hz_ = declare_parameter<double>("update_rate_hz", 100.0);
    pressed_ = declare_parameter<bool>("pressed", true);

    if (update_rate_hz_ <= 0.0) {
      throw std::runtime_error("update_rate_hz must be positive");
    }

    if (can_id_ < 0 || can_id_ > 0x1FFFFFFF) {
      throw std::runtime_error("can_id out of range");
    }

    bus_ = CanBusRegistry::get(can_iface_);
    if (!bus_) {
      throw std::runtime_error("Failed to acquire CAN bus on interface '" +
                               can_iface_ + "'");
    }

    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / update_rate_hz_));
    timer_ =
        create_wall_timer(period, std::bind(&MockLimitSwitchNode::publish, this));

    RCLCPP_INFO(get_logger(),
                "Mock limit switch spamming %s state on %s id 0x%X at %.1f Hz",
                pressed_ ? "HIGH" : "LOW", can_iface_.c_str(), can_id_,
                update_rate_hz_);
  }

private:
  void publish() {
    struct can_frame frame {};
    frame.can_id = static_cast<uint32_t>(can_id_) & 0x1FFFFFFFu;
    frame.can_dlc = 1;
    frame.data[0] = pressed_ ? 0x1 : 0x0;
    bus_->enqueue_tx(frame);
  }

  std::shared_ptr<CanBusManager> bus_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::string can_iface_;
  int can_id_{0x181};
  double update_rate_hz_{100.0};
  bool pressed_{true};
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  int ret = 0;
  try {
    rclcpp::spin(std::make_shared<MockLimitSwitchNode>());
  } catch (const std::exception &ex) {
    RCLCPP_FATAL(rclcpp::get_logger("mock_limit_switch"),
                 "Unhandled exception: %s", ex.what());
    ret = EXIT_FAILURE;
  }
  rclcpp::shutdown();
  return ret;
}
