#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>

#include <linux/can.h>

#include "geometry_msgs/msg/vector3.hpp"
#include "mr2_can_bus_core/can_bus_registry.hpp"
#include "rclcpp/rclcpp.hpp"

namespace {
constexpr uint32_t kStdIdMask = 0x7FF;
constexpr uint16_t kAdcCenter = 2048;
constexpr uint16_t kAdcMax = 4095;

uint16_t normalized_to_adc(double value) {
  if (!std::isfinite(value)) {
    return kAdcCenter;
  }
  const double clamped = std::clamp(value, -1.0, 1.0);
  return static_cast<uint16_t>(
      std::lround(((clamped + 1.0) * 0.5) * kAdcMax));
}

} // namespace

class CameraTurretCanNode : public rclcpp::Node {
public:
  CameraTurretCanNode() : rclcpp::Node("camera_turret_can") {
    can_iface_ = declare_parameter<std::string>("can_iface", "can0");
    const int can_id_param = declare_parameter<int>("can_id", 0x124);
    command_topic_ =
        declare_parameter<std::string>("command_topic", "/camera_turret/command");
    const double publish_rate_hz =
        declare_parameter<double>("publish_rate_hz", 50.0);
    invert_x_ = declare_parameter<bool>("invert_x", false);
    invert_y_ = declare_parameter<bool>("invert_y", false);

    if (can_id_param < 0 || can_id_param > static_cast<int>(kStdIdMask)) {
      throw std::runtime_error("Invalid CAN ID: " +
                               std::to_string(can_id_param));
    }
    if (publish_rate_hz <= 0.0 || !std::isfinite(publish_rate_hz)) {
      throw std::runtime_error("publish_rate_hz must be finite and positive");
    }
    can_id_ = static_cast<uint32_t>(can_id_param);
    bus_ = CanBusRegistry::get(can_iface_);
    if (!bus_) {
      throw std::runtime_error("Failed to acquire CAN bus on " + can_iface_);
    }

    command_sub_ = create_subscription<geometry_msgs::msg::Vector3>(
        command_topic_, rclcpp::SystemDefaultsQoS(),
        [this](const geometry_msgs::msg::Vector3::SharedPtr msg) {
          latest_x_ = msg->x;
          latest_y_ = msg->y;
          have_command_ = true;
        });

    const auto timer_period =
        std::chrono::duration<double>(1.0 / publish_rate_hz);
    publish_timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(timer_period),
        [this]() { publish_can_command(); });

    RCLCPP_INFO(get_logger(),
                "camera_turret_can ready on %s, classic CAN StdID 0x%03X, "
                "topic %s",
                can_iface_.c_str(), can_id_, command_topic_.c_str());
  }

  ~CameraTurretCanNode() override {
    send_can_command(0.0, 0.0, true);
  }

private:
  void publish_can_command() {
    send_can_command(have_command_ ? latest_x_ : 0.0,
                     have_command_ ? latest_y_ : 0.0, false);
  }

  void send_can_command(double x, double y, bool final_command) {
    if (invert_x_) {
      x = -x;
    }
    if (invert_y_) {
      y = -y;
    }

    const uint16_t vrx = normalized_to_adc(y);
    const uint16_t vry = normalized_to_adc(x);

    struct can_frame frame {};
    frame.can_id = can_id_ & kStdIdMask;
    frame.can_dlc = 4;
    frame.data[0] = static_cast<uint8_t>(vrx & 0xFF);
    frame.data[1] = static_cast<uint8_t>((vrx >> 8) & 0xFF);
    frame.data[2] = static_cast<uint8_t>(vry & 0xFF);
    frame.data[3] = static_cast<uint8_t>((vry >> 8) & 0xFF);
    if (final_command) {
      bus_->transmit_last(frame);
    } else {
      bus_->enqueue_tx(frame);
    }
  }

  std::string can_iface_;
  std::string command_topic_;
  uint32_t can_id_{0};
  std::shared_ptr<CanBusManager> bus_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr command_sub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;

  double latest_x_{0.0};
  double latest_y_{0.0};
  bool have_command_{false};
  bool invert_x_{false};
  bool invert_y_{false};
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<CameraTurretCanNode>();
    rclcpp::spin(node);
  } catch (const std::exception &ex) {
    RCLCPP_FATAL(rclcpp::get_logger("camera_turret_can"), "%s", ex.what());
  }
  rclcpp::shutdown();
  return 0;
}
